import AVFoundation
import Foundation
import Observation
import simd

/// Everything the capture-queue frame handler needs, detached from the
/// MainActor view model. Built once per session; @unchecked Sendable because
/// mutable state is guarded by `lock` and the rest is immutable or actors.
final class FrameFeedContext: @unchecked Sendable {
    /// The band one capture should land in: **200 is the minimum worth
    /// training a room from, 500 is the ceiling.**
    ///
    /// This bounds a single capture, not a project. A house is a chain of
    /// sessions — around ten captures of 200-500 frames — sharing one world
    /// frame, so the project total is bounded by free disk, not by this.
    ///
    /// Comfortably under the final solve's memory ceiling too, which is a
    /// separate limit for a separate reason: the solve holds every frame's
    /// features resident, measured at 1.24 MB/frame. See
    /// docs/ARCHITECTURE.md, "How big a project can get".
    static let storedFrameCap = 500
    static let storedFrameWarn = 450
    static let storedFrameTarget = 200

    /// Motion gate, matching the engine's own store thresholds
    /// (`store_min_translation_m`, `store_min_rotation_deg`). Storage exists
    /// to give the final solve NEW viewpoints; a frame taken from where the
    /// last one was taken is a duplicate, and with a 500-frame budget a
    /// duplicate is not free — it is a viewpoint somewhere else that never
    /// gets captured.
    static let storeMinTranslationM = 0.05
    static let storeMinRotationDeg = 5.0
    /// Store anyway after this long. Covers the case where there is no pose
    /// to compare against (tracking lost) and the case of genuinely slow,
    /// deliberate movement — when in doubt, keep the frame.
    static let storeMaxIntervalS = 3.0

    /// Refuse to start storing when the device is this close to full. A
    /// capture that dies on a write halfway through a room is worse than one
    /// that never starts, and "your phone is full" is something the user can
    /// act on before walking the space rather than after.
    static let minFreeBytes: Int64 = 2 * 1024 * 1024 * 1024

    let store: SessionStore
    let encoder = JpegEncoder()
    let encodeQueue = DispatchQueue(label: "bs.jpeg", qos: .utility)
    let previewRenderer: VideoPreviewRenderer
    let videoDims: (width: Int, height: Int)

    let lock = NSLock()
    var nextFrameId: UInt32 = 1
    var lastStoreTime: Double = -1
    var encodesInFlight = 0
    var storeCommits = 0
    /// Frames stored so far by the scout circuit. Split out from
    /// `storeCommits` only so the UI can say which half of the budget the
    /// user has spent — both halves share one session and one disk cap.
    var scoutCommits = 0

    /// Latest live pose, published by the polling loop for the storage gate.
    /// Guarded by `lock`: written on the main actor, read on the capture
    /// queue. Up to ~100 ms stale, which at walking pace is a few
    /// centimetres — fine for a 5 cm decision, and stale in the safe
    /// direction (it under-reports movement, so it stores more rather than
    /// less).
    private var latestCentre: SIMD3<Double>?
    private var latestRotation: simd_quatd?
    /// Pose of the last frame actually written, to measure movement against.
    private var lastStoredCentre: SIMD3<Double>?
    private var lastStoredRotation: simd_quatd?

    func publishPose(_ viewer: ViewerPose?) {
        lock.lock()
        latestCentre = viewer?.center
        latestRotation = viewer?.rotation
        lock.unlock()
    }

    /// "capture" or "scout", stamped into each frame's meta as it is written.
    /// Read and written under `lock` because the pass flips on the main
    /// actor while the capture queue is mid-frame.
    private var passName = "capture"
    var pass: String {
        get { lock.lock(); defer { lock.unlock() }; return passName }
        set { lock.lock(); passName = newValue; lock.unlock() }
    }

    init(store: SessionStore, previewRenderer: VideoPreviewRenderer,
         videoDims: (width: Int, height: Int)) {
        self.store = store
        self.previewRenderer = previewRenderer
        self.videoDims = videoDims
    }

    /// Runs on the capture queue for every synchronized frame. Budget is the
    /// 33 ms frame interval; the heavy JPEG encode is bounced to encodeQueue.
    func handle(_ frame: CapturedFrame) {
        let stats = FrameAnalysis.lumaStats(of: frame.pixelBuffer)
        guard let depth = DepthPacker.pack(frame.depthData.depthDataMap) else { return }

        lock.lock()
        let frameId = nextFrameId
        nextFrameId += 1
        lock.unlock()

        feedEngine(frame, frameId: frameId, depth: depth)
        previewRenderer.enqueue(frame.pixelBuffer)

        // Storage gate: cadence floor, exposure sanity, and MOVEMENT.
        //
        // The movement term is what makes a 500-frame budget go far. On a
        // pure clock this stored 3.3 frames every second the camera was
        // running, so pausing to think, or turning on the spot to look at
        // something, spent the room's budget on views the solve already had.
        // Storage exists to give the final solve new viewpoints; a frame from
        // where the last one was taken is a duplicate, and a duplicate is not
        // free — it is a viewpoint elsewhere that never gets captured.
        //
        // Raw storage is still never dropped for backpressure: the gate only
        // thins cadence when the encoder cannot keep up, and that surfaces in
        // the UI meter.
        lock.lock()
        let elapsed = frame.tCapture - lastStoreTime
        let busy = encodesInFlight >= 2
        let capped = storeCommits >= Self.storedFrameCap
        // No pose (tracking lost, or before bootstrap) means we cannot tell
        // whether anything changed — so keep the frame. Losing a real
        // viewpoint is worse than keeping a redundant one.
        var moved = true
        if let centre = latestCentre, let rotation = latestRotation,
           let lastCentre = lastStoredCentre, let lastRotation = lastStoredRotation {
            let step = simd_distance(centre, lastCentre)
            // Angle between two orientations is 2·acos(|q0·q1|). Taken on the
            // underlying 4-vectors, and abs() because q and -q are the same
            // rotation — without it a sign flip reads as a 180 degree turn.
            let dot = abs(simd_dot(rotation.vector, lastRotation.vector))
            let turn = 2 * acos(min(1.0, dot)) * 180 / .pi
            moved = step >= Self.storeMinTranslationM
                || turn >= Self.storeMinRotationDeg
                || elapsed >= Self.storeMaxIntervalS
        }
        let shouldStore = !busy && !capped && elapsed >= 0.30 && moved
            && stats.overexposedFraction < 0.10
        let framePass = passName
        if shouldStore {
            lastStoreTime = frame.tCapture
            lastStoredCentre = latestCentre
            lastStoredRotation = latestRotation
            encodesInFlight += 1
            storeCommits += 1
            if framePass == "scout" { scoutCommits += 1 }
        }
        lock.unlock()
        guard shouldStore else { return }

        let meta = FrameMetaJSON(
            frameId: frameId,
            tCapture: frame.tCapture,
            tDepth: frame.tDepth,
            intrinsics: Self.scaledIntrinsics(
                frame.calibration, toWidth: videoDims.width, height: videoDims.height),
            depthIntrinsics: Self.scaledIntrinsics(
                frame.calibration, toWidth: depth.width, height: depth.height),
            exposure: ExposureJSON(
                durationS: frame.exposureDuration, iso: frame.iso,
                biasEv: frame.exposureBias),
            quality: QualityJSON(
                lapVar: stats.laplacianVariance,
                overexpFrac: stats.overexposedFraction),
            isKeyframe: false,
            storeReason: "gate",
            pass: framePass)

        // The calibrator needs a frame that is genuinely going to disk: it
        // records the floor against a frame id, and the solve resolves that
        // through the frame's final pose. Publishing it only on the stored
        // path is what keeps that guarantee.
        //
        // Gated, because handing over the depth copies a third of a
        // megabyte, and the floor is measured once in the first seconds of
        // a session that may run for minutes.
        if wantsFloorFrames {
            onStoredFrame?(frameId, depth.f32, Int32(depth.width),
                           Int32(depth.height),
                           Self.scaledIntrinsics(frame.calibration,
                                                 toWidth: depth.width,
                                                 height: depth.height))
        }

        let pixelBuffer = frame.pixelBuffer  // retained by the closure
        let calibration = frame.calibration
        encodeQueue.async { [self] in
            defer {
                lock.lock()
                encodesInFlight -= 1
                lock.unlock()
            }
            guard let jpeg = encoder.encode(pixelBuffer) else {
                lock.lock()
                storeCommits -= 1  // failed encode gives its cap slot back
                lock.unlock()
                return
            }
            let payload = SessionStore.FramePayload(
                frameId: frameId, jpeg: jpeg, depthF16: depth.f16,
                depthWidth: depth.width, depthHeight: depth.height, meta: meta)
            Task { [store] in
                if let calibration {
                    try? await store.writeCalibrationIfNeeded(calibration)
                }
                try? await store.writeFrame(payload)
            }
        }
    }

    /// Called on the capture queue for each frame that is being stored,
    /// with its depth and depth intrinsics. Used by the floor calibration.
    var onStoredFrame: ((UInt32, [Float], Int32, Int32,
                         PinholeIntrinsicsJSON) -> Void)?
    /// Cleared once the floor is measured or skipped, so the depth copy
    /// above stops for the rest of the session.
    var wantsFloorFrames = false

    private func feedEngine(_ frame: CapturedFrame, frameId: UInt32,
                            depth: DepthPacker.Packed) {
        CVPixelBufferLockBaseAddress(frame.pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(frame.pixelBuffer, .readOnly) }
        guard let lumaBase = CVPixelBufferGetBaseAddressOfPlane(frame.pixelBuffer, 0)
        else { return }

        let width = CVPixelBufferGetWidthOfPlane(frame.pixelBuffer, 0)
        let height = CVPixelBufferGetHeightOfPlane(frame.pixelBuffer, 0)
        let stride = CVPixelBufferGetBytesPerRowOfPlane(frame.pixelBuffer, 0)

        var input = bs_frame_in()
        input.frame_id = frameId
        input.t_capture = frame.tCapture
        input.t_depth = frame.tDepth
        input.luma = UnsafePointer(lumaBase.assumingMemoryBound(to: UInt8.self))
        input.luma_width = Int32(width)
        input.luma_height = Int32(height)
        input.luma_stride = Int32(stride)

        if let calibration = frame.calibration {
            let m = calibration.intrinsicMatrix
            let ref = calibration.intrinsicMatrixReferenceDimensions
            let sx = Double(width) / Double(ref.width)
            let sy = Double(height) / Double(ref.height)
            input.fx = Double(m.columns.0.x) * sx
            input.fy = Double(m.columns.1.y) * sy
            input.cx = Double(m.columns.2.x) * sx
            input.cy = Double(m.columns.2.y) * sy
            let dsx = Double(depth.width) / Double(ref.width)
            let dsy = Double(depth.height) / Double(ref.height)
            input.dfx = Double(m.columns.0.x) * dsx
            input.dfy = Double(m.columns.1.y) * dsy
            input.dcx = Double(m.columns.2.x) * dsx
            input.dcy = Double(m.columns.2.y) * dsy
        }
        if let gyro = frame.gyroDelta {
            input.gyro_dx = gyro.x
            input.gyro_dy = gyro.y
            input.gyro_dz = gyro.z
            input.gyro_valid = 1
        }

        depth.f32.withUnsafeBufferPointer { depthBuf in
            input.depth = depthBuf.baseAddress
            input.depth_width = Int32(depth.width)
            input.depth_height = Int32(depth.height)
            _ = CoreEngine.shared.liveFeed(&input)
        }
    }

    static func scaledIntrinsics(
        _ calibration: AVCameraCalibrationData?, toWidth w: Int, height h: Int
    ) -> PinholeIntrinsicsJSON {
        guard let calibration else {
            return PinholeIntrinsicsJSON(fx: 0, fy: 0, cx: 0, cy: 0, refW: w, refH: h)
        }
        let m = calibration.intrinsicMatrix
        let ref = calibration.intrinsicMatrixReferenceDimensions
        let sx = Double(w) / Double(ref.width)
        let sy = Double(h) / Double(ref.height)
        return PinholeIntrinsicsJSON(
            fx: Double(m.columns.0.x) * sx, fy: Double(m.columns.1.y) * sy,
            cx: Double(m.columns.2.x) * sx, cy: Double(m.columns.2.y) * sy,
            refW: w, refH: h)
    }
}

/// Orchestrates one capture session and publishes UI state. All members are
/// MainActor; per-frame work lives in FrameFeedContext on capture threads.
@Observable
@MainActor
final class CaptureViewModel {
    enum State: Equatable {
        case idle
        case starting
        /// Walking the opening circuit. Frames are stored and tagged
        /// `pass="scout"`; the engine is building the localization scaffold.
        case scouting
        case capturing
        case stopping
        case finished(sessionName: String)
        case failed(String)
    }

    /// Whether the session opens with a scout circuit. Offered before the
    /// camera starts, because it changes what the first minute is for.
    enum Plan: Equatable {
        /// Walk the space once first, then scan. The circuit builds a
        /// scaffold so the detailed pass always has something to hold
        /// position against — worth its minute in anything bigger than a
        /// single room.
        case scoutThenCapture
        /// Straight into detailed capture.
        case captureOnly
    }

    private(set) var state: State = .idle
    private(set) var plan: Plan = .captureOnly
    /// Set before start() to add this capture to an existing project rather
    /// than beginning a new one.
    var continuingProject: ProjectStore.Project?
    /// Name for a brand-new project. Ignored when continuing one.
    var newProjectName: String?
    private(set) var guidance = "Preparing…"
    private(set) var framesSeen: UInt32 = 0
    private(set) var framesStored: UInt32 = 0
    private(set) var megabytesWritten: Double = 0
    private(set) var storageNote: String?
    private(set) var readinessOverall: Float = 0
    private(set) var snapshot = CoreEngine.Snapshot()
    /// Nil once the floor step is done or skipped; drives the prompt.
    private(set) var floorPhase: FloorCalibrator.Phase?
    /// Frames the scout circuit stored, frozen when the circuit ends.
    private(set) var scoutFramesStored: UInt32 = 0
    /// Keyframes in the scaffold the scout circuit left behind. This is the
    /// number that says whether the circuit was worth walking — a lap that
    /// produced almost no keyframes did not map anything to localize into.
    private(set) var scaffoldKeyframes: UInt32 = 0

    /// Where the user is standing right now, when the tracker knows. Every
    /// distance and direction the UI quotes goes through this — the readiness
    /// grid works in the session's world frame, whose origin is wherever the
    /// first keyframe landed, and a distance measured from there reads as a
    /// distance from the user while being nothing of the kind.
    private(set) var viewer: ViewerPose?

    /// Set only while tracking is lost and the engine knows which way the
    /// map is. Drives the recovery arrow.
    private(set) var recovery: RecoveryHint?

    var isScouting: Bool { state == .scouting }

    private let floorCalibrator = FloorCalibrator()

    private let manager = CaptureManager()
    private var context: FrameFeedContext?
    private var pollTask: Task<Void, Never>?
    private var snapshotTick = 0

    let previewRenderer = VideoPreviewRenderer()
    let mapRenderer = MapRenderer()

    var isCapturing: Bool { state == .capturing }
    /// Both passes feed the engine and write frames; several call sites care
    /// only about "is the camera running".
    var isRunning: Bool { state == .capturing || state == .scouting }

    func start(plan: Plan = .captureOnly) {
        switch state {
        case .idle, .finished, .failed:
            self.plan = plan
            state = .starting
            Task { await startInner() }
        default:
            break
        }
    }

    private func startInner() async {
        let granted = await AVCaptureDevice.requestAccess(for: .video)
        guard granted else {
            state = .failed("Camera access denied. Enable it in Settings → BetterSplats.")
            return
        }

        do {
            try manager.configure()
        } catch {
            state = .failed(error.localizedDescription)
            return
        }

        let store: SessionStore
        do {
            store = try SessionStore(
                videoW: manager.videoDimensions.width,
                videoH: manager.videoDimensions.height,
                depthW: manager.depthDimensions.width,
                depthH: manager.depthDimensions.height,
                fps: 30,
                continuing: continuingProject,
                newProjectName: newProjectName)
        } catch {
            state = .failed("Could not create session storage: \(error)")
            return
        }

        let engine = CoreEngine.shared
        let firstPass: bs_pass_kind =
            plan == .scoutThenCapture ? BS_PASS_SCOUT : BS_PASS_CAPTURE
        guard engine.liveBegin(sessionDir: store.directory.path,
                               pass: firstPass) == BS_OK else {
            state = .failed("Engine rejected session: \(engine.lastError)")
            return
        }

        let context = FrameFeedContext(
            store: store, previewRenderer: previewRenderer,
            videoDims: manager.videoDimensions)
        // Continue the project's numbering. Restarting at 1 in every capture
        // would give two captures the same frame ids, and a chain with
        // duplicates is rejected outright rather than silently resolving to
        // one of them.
        context.nextFrameId = await store.firstFrameId
        context.pass = plan == .scoutThenCapture ? "scout" : "capture"
        self.context = context
        manager.onFrame = { frame in
            context.handle(frame)  // capture queue
        }

        if plan == .captureOnly { beginFloorCalibration(context) }

        manager.start()
        state = plan == .scoutThenCapture ? .scouting : .capturing
        guidance = plan == .scoutThenCapture
            ? "Walk the space — back to the walls, camera facing in"
            : "Move slowly and keep the scene in view"
        startPolling()
    }

    /// Ends the scout circuit and starts detailed capture in the same
    /// session. The engine writes `live/map.bin` on the scout `liveEnd`, and
    /// the capture `liveBegin` loads it, so both passes share one world
    /// frame — which is the whole point of walking the circuit.
    func finishScout() {
        guard state == .scouting, let context else { return }
        state = .starting
        Task {
            // Let in-flight encodes drain so the scaffold's frames are all on
            // disk before the pass boundary.
            try? await Task.sleep(for: .milliseconds(400))
            scoutFramesStored = await context.store.storedFrames

            let engine = CoreEngine.shared
            engine.liveEnd()
            scaffoldKeyframes = engine.livePollStatus().keyframes

            guard engine.liveBegin(sessionDir: context.store.directory.path,
                                   pass: BS_PASS_CAPTURE) == BS_OK else {
                state = .failed("Engine rejected capture pass: \(engine.lastError)")
                return
            }
            context.pass = "capture"

            // Only now. The floor calibration names a frame id and the solve
            // resolves it through that frame's final pose — but the solve
            // excludes scout frames outright, so a floor measured during the
            // circuit would point at a frame that never gets a pose, and be
            // silently dropped.
            beginFloorCalibration(context)

            state = .capturing
            guidance = "Move slowly and keep the scene in view"
        }
    }

    /// Floor calibration runs over the first stored frames of the CAPTURE
    /// pass. It is part of the ordinary capture stream on purpose — a
    /// calibration frame has to be one the final solve registers like any
    /// other, and a separate mode would be free to produce frames that never
    /// do.
    private func beginFloorCalibration(_ context: FrameFeedContext) {
        floorPhase = .aiming(advice: "Point at the floor and take a step",
                             heightM: nil)
        context.wantsFloorFrames = true
        context.onStoredFrame = { [weak self] frameId, depthF32, w, h, K in
            Task { @MainActor in
                guard let self, let store = self.context?.store else { return }
                guard let accepted = self.floorCalibrator.offer(
                    depthF32: depthF32, width: w, height: h,
                    fx: K.fx, fy: K.fy, cx: K.cx, cy: K.cy,
                    storedFrameId: frameId) else {
                    // The calibrator gives up on its own after a while. That
                    // path has to end the floor step properly, or the camera
                    // never locks and the depth copy runs all session.
                    if self.floorCalibrator.isComplete {
                        // Gave up on its own. Say so rather than letting the
                        // prompt vanish — someone who was following an
                        // instruction deserves to know it was dropped, and
                        // that the scan is fine without it.
                        self.floorPhase = .skipped
                        self.context?.wantsFloorFrames = false
                        self.endFloorStep()
                        Task { @MainActor in
                            try? await Task.sleep(nanoseconds: 2_000_000_000)
                            self.floorPhase = nil
                        }
                    } else {
                        self.floorPhase = self.floorCalibrator.phase
                    }
                    return
                }
                await store.setFloorCalibration(
                    frameId: accepted.frameId, normal: accepted.normal,
                    offsetM: accepted.heightM, rmseM: accepted.rmseM,
                    incidenceDeg: accepted.incidenceDeg,
                    inliers: accepted.inliers)
                self.floorPhase = self.floorCalibrator.phase
                self.context?.wantsFloorFrames = false
                self.endFloorStep()
                // Let the confirmation land, then get out of the way.
                Task { @MainActor in
                    try? await Task.sleep(nanoseconds: 1_600_000_000)
                    self.floorPhase = nil
                }
            }
        }
    }

    /// The floor step is over, one way or another. This is the moment the
    /// camera is finally pointed at the scene the user is going to scan, so
    /// it is the moment to converge AF/AE/AWB and freeze them — not session
    /// start, when the phone is aimed at the floor because we asked it to be.
    private func endFloorStep() {
        guard !photometryLocked else { return }
        photometryLocked = true
        manager.settleThenLock()
    }
    private var photometryLocked = false

    private func startPolling() {
        pollTask?.cancel()
        pollTask = Task { [weak self] in
            while !Task.isCancelled {
                try? await Task.sleep(for: .milliseconds(100))
                guard let self, self.isRunning, let context = self.context
                else { continue }
                let status = CoreEngine.shared.livePollStatus()
                self.viewer = ViewerPose(status: status)
                // The storage gate needs this on the capture queue.
                context.publishPose(self.viewer)
                self.recovery = RecoveryHint(status: status)
                self.framesSeen = status.frames_fed
                // The recovery hint is a better version of the same message,
                // so it replaces the generic pill rather than sitting beside
                // it repeating itself.
                self.guidance = self.recovery?.text
                    ?? (self.isScouting
                        ? Self.scoutGuidanceText(for: status)
                        : Self.guidanceText(for: status))
                if self.isScouting { self.scaffoldKeyframes = status.keyframes }
                self.readinessOverall = status.readiness_overall
                self.framesStored = await context.store.storedFrames
                self.megabytesWritten =
                    Double(await context.store.storedBytes) / 1_048_576.0
                let stored = Int(self.framesStored)
                let free = Self.freeDiskBytes()
                if free < FrameFeedContext.minFreeBytes {
                    // Disk beats the frame count: a capture that dies on a
                    // write halfway through a room loses the room.
                    self.storageNote = String(
                        format: "iPhone almost full (%.1f GB left) — stop soon",
                        Double(free) / 1_073_741_824)
                } else if stored >= FrameFeedContext.storedFrameCap {
                    self.storageNote =
                        "Capture full — stop here, then continue this project "
                        + "in a new capture"
                } else if stored >= FrameFeedContext.storedFrameWarn {
                    self.storageNote = "Nearly full: \(stored)/"
                        + "\(FrameFeedContext.storedFrameCap) — finish up"
                } else if stored < FrameFeedContext.storedFrameTarget {
                    // The under-covered case is the one people actually hit,
                    // and it is invisible without being told: the capture
                    // looks fine, and the thinness only shows up as holes in
                    // the trained splat hours later.
                    self.storageNote = "\(stored)/"
                        + "\(FrameFeedContext.storedFrameTarget) frames — keep "
                        + "going for full coverage"
                } else {
                    self.storageNote = nil
                }
                self.snapshotTick += 1
                if self.snapshotTick % 5 == 0 {  // ~2 Hz snapshot refresh
                    self.refreshSnapshot()
                }
            }
        }
    }

    /// Free space on the volume the sessions live on. 0 when it cannot be
    /// read, which reads as "almost full" and warns — the safe direction,
    /// since the alternative is discovering it mid-capture.
    private static func freeDiskBytes() -> Int64 {
        let url = FileManager.default.urls(for: .documentDirectory,
                                           in: .userDomainMask)[0]
        let values = try? url.resourceValues(
            forKeys: [.volumeAvailableCapacityForImportantUsageKey])
        return values?.volumeAvailableCapacityForImportantUsage ?? 0
    }

    func refreshSnapshot() {
        snapshot = CoreEngine.shared.snapshot()
        mapRenderer.update(with: snapshot)
    }

    func renameRegion(id: UInt32, name: String) {
        _ = CoreEngine.shared.renameRegion(id: id, name: name)
        refreshSnapshot()
    }

    private static func guidanceText(for status: bs_live_status) -> String {
        switch bs_guidance(rawValue: bs_guidance.RawValue(max(0, status.guidance))) {
        case BS_GUIDE_GOOD: return "Good — keep going"
        case BS_GUIDE_MOVE_CLOSER: return "Move closer"
        case BS_GUIDE_MOVE_SIDEWAYS: return "Step sideways to add parallax"
        case BS_GUIDE_SLOW_DOWN: return "Slow down"
        case BS_GUIDE_RECAPTURE: return "Recapture this area"
        case BS_GUIDE_TRACKING_LOST: return "Tracking lost — return to a mapped area"
        case BS_GUIDE_COVERAGE_NEEDED: return "More coverage needed here"
        default: return "Capturing"
        }
    }

    /// The circuit is a different job from detailed capture, so most of the
    /// detail-capture advice is wrong here. "Move closer" and "more coverage
    /// needed" would send someone to do the scan they have not started yet;
    /// the circuit wants distance and speed, not proximity and dwell. Only
    /// the two failures that actually break a scaffold are surfaced.
    private static func scoutGuidanceText(for status: bs_live_status) -> String {
        switch bs_guidance(rawValue: bs_guidance.RawValue(max(0, status.guidance))) {
        case BS_GUIDE_SLOW_DOWN:
            return "Turn more slowly"
        case BS_GUIDE_TRACKING_LOST:
            return "Lost the map — go back the way you came"
        default:
            return "Keep walking — back to the walls, camera facing in"
        }
    }

    /// The user chose to start scanning without measuring the floor.
    /// Levelling then infers it from the reconstruction, which works — it is
    /// simply less certain, and can decline on an ambiguous scan.
    func skipFloorCalibration() {
        floorCalibrator.skip()
        context?.wantsFloorFrames = false
        floorPhase = nil
        endFloorStep()
    }

    func stop() {
        guard state == .capturing || state == .scouting else { return }
        state = .stopping
        pollTask?.cancel()
        manager.onFrame = nil
        manager.stop()

        Task {
            // Let in-flight encodes drain before finalizing.
            try? await Task.sleep(for: .milliseconds(600))
            CoreEngine.shared.liveEnd()
            if let context {
                let locks = manager.lockState
                try? await context.store.finalize(
                    locks: (focus: locks.focus, exposure: locks.exposure,
                            whiteBalance: locks.whiteBalance))
                let name = context.store.directory.lastPathComponent
                state = .finished(sessionName: name)
                guidance = "Saved \(name)"
            } else {
                state = .idle
            }
        }
    }
}

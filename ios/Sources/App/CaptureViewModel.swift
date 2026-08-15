import AVFoundation
import Foundation
import SwiftUI
import UIKit
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

    /// Motion gate. Storage exists to give the final solve NEW viewpoints; a
    /// frame taken from where the last one was taken is a duplicate, and with
    /// a 500-frame budget a duplicate is not free — it is a viewpoint
    /// somewhere else that never gets captured.
    ///
    /// The SPACING comes from the engine, live, through
    /// `bs_live_status.store_spacing_m`: how far the camera must travel
    /// before another frame is worth having depends on how far away what it
    /// is looking at is, and the engine is the only side that knows. These
    /// constants are the floor for before the engine has a scene to measure.
    ///
    /// This used to be the app's own copy of the rule, described as
    /// "matching the engine's own store thresholds" — and when the engine's
    /// gate became depth-scaled, it stopped matching. The app would have gone
    /// on storing at a flat 5 cm and the change would have done nothing on
    /// device, which is the same shape of defect as `keyframe_ids` being
    /// empty in every session ever captured.
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
    /// Runs the live tracker off the capture queue. See EngineFeeder for why
    /// feeding inline cost frames the app never even saw.
    let feeder = EngineFeeder()
    let previewRenderer: VideoPreviewRenderer
    let videoDims: (width: Int, height: Int)

    let lock = NSLock()
    var nextFrameId: UInt32 = 1
    var lastStoreTime: Double = -1
    /// Frames somewhere between "handed to the encoder" and "on disk". The
    /// cap on this is the storage backpressure: past it the gate thins
    /// cadence rather than queueing work the device cannot keep up with.
    var encodesInFlight = 0
    /// Depth of that pipeline. Three rather than two because a slot now spans
    /// the write as well as the encode, and the encoder should not sit idle
    /// while one frame drains to flash.
    static let maxEncodesInFlight = 3
    var storeCommits = 0
    /// Frames that failed to reach disk. Non-zero means the capture on disk
    /// is smaller than the session thinks it walked, which changes what the
    /// end-of-capture verdict should say.
    var storeFailures = 0
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
    /// Required spacing between stored frames, published by the engine each
    /// poll. Guarded by `lock` like the pose it travels with.
    private var storeSpacingM: Double = 0
    /// Pose of the last frame actually written, to measure movement against.
    private var lastStoredCentre: SIMD3<Double>?
    private var lastStoredRotation: simd_quatd?

    func publishPose(_ viewer: ViewerPose?, storeSpacingM spacing: Double = 0) {
        lock.lock()
        latestCentre = viewer?.center
        latestRotation = viewer?.rotation
        // 0 means the engine has no scene depth yet — keep the last spacing
        // rather than snapping back to the floor and storing a burst.
        if spacing > 0 { storeSpacingM = spacing }
        lock.unlock()
    }

    /// Frames the writer could not put on disk. Read from the main actor each
    /// poll; written on the storage path.
    var failedWrites: Int {
        lock.lock(); defer { lock.unlock() }; return storeFailures
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

        // Preview first, then the engine. The order matters: the feed is
        // asynchronous now, but the preview is what the user is looking at and
        // it should never queue behind anything.
        previewRenderer.enqueue(frame.pixelBuffer)
        feedEngine(frame, frameId: frameId, depth: depth)

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
        // Backpressure THINS storage, it does not queue it. When the pipeline
        // is full this frame is skipped, and because the movement gate is a
        // distance rather than a clock, the next frame is still past the
        // threshold — so what is lost is cadence, not coverage. Frames that
        // fail to write are a different matter and are counted, not skipped
        // silently.
        lock.lock()
        let elapsed = frame.tCapture - lastStoreTime
        let busy = encodesInFlight >= Self.maxEncodesInFlight
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
            moved = step >= max(Self.storeMinTranslationM, storeSpacingM)
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
            guard let jpeg = encoder.encode(pixelBuffer) else {
                lock.lock()
                encodesInFlight -= 1
                storeCommits -= 1  // failed encode gives its cap slot back
                lock.unlock()
                return
            }
            let payload = SessionStore.FramePayload(
                frameId: frameId, jpeg: jpeg, depthF16: depth.f16,
                depthWidth: depth.width, depthHeight: depth.height, meta: meta)
            Task { [store] in
                // The in-flight slot is released HERE, after the write, not
                // when the encode returned. Releasing it early made the
                // backpressure gate blind to the slow half: encoding is
                // hardware and quick, writing 1.3 MB to flash is neither, and
                // every encoded frame span=ned an unstructured Task holding its
                // payload. Nothing bounded those, so a stalled disk grew
                // memory by a megabyte a frame instead of thinning cadence.
                defer {
                    lock.lock()
                    encodesInFlight -= 1
                    lock.unlock()
                }
                do {
                    if let calibration {
                        try await store.writeCalibrationIfNeeded(calibration)
                    }
                    try await store.writeFrame(payload)
                } catch {
                    // A frame that did not land must not be counted as one
                    // that did: `storeCommits` is the cap, and `storedFrames`
                    // is what the end-of-capture verdict judges the session
                    // on. Silently keeping the count would tell the user they
                    // had 400 frames when the disk took 300.
                    lock.lock()
                    storeCommits -= 1
                    storeFailures += 1
                    lock.unlock()
                }
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

    /// Copies the frame out and hands it to the feeder, which calls the engine
    /// on its own queue. Everything the engine needs is owned by the copy: the
    /// capture callback's pixel buffers die when it returns.
    private func feedEngine(_ frame: CapturedFrame, frameId: UInt32,
                            depth: DepthPacker.Packed) {
        CVPixelBufferLockBaseAddress(frame.pixelBuffer, .readOnly)
        let luma = EngineFeeder.downscaledLuma(from: frame.pixelBuffer)
        CVPixelBufferUnlockBaseAddress(frame.pixelBuffer, .readOnly)
        guard let luma else { return }

        var out = EngineFeeder.Frame(
            frameId: frameId, tCapture: frame.tCapture, tDepth: frame.tDepth,
            luma: luma.pixels, lumaWidth: luma.width, lumaHeight: luma.height,
            fx: 0, fy: 0, cx: 0, cy: 0,
            depth: depth.f32, depthWidth: depth.width, depthHeight: depth.height,
            dfx: 0, dfy: 0, dcx: 0, dcy: 0,
            gyro: frame.gyroDelta.map { SIMD3($0.x, $0.y, $0.z) })

        if let calibration = frame.calibration {
            let m = calibration.intrinsicMatrix
            let ref = calibration.intrinsicMatrixReferenceDimensions
            // Scale to the size of the image actually being handed over, not
            // to the full-resolution plane it came from.
            let sx = Double(luma.width) / Double(ref.width)
            let sy = Double(luma.height) / Double(ref.height)
            out.fx = Double(m.columns.0.x) * sx
            out.fy = Double(m.columns.1.y) * sy
            out.cx = Double(m.columns.2.x) * sx
            out.cy = Double(m.columns.2.y) * sy
            let dsx = Double(depth.width) / Double(ref.width)
            let dsy = Double(depth.height) / Double(ref.height)
            out.dfx = Double(m.columns.0.x) * dsx
            out.dfy = Double(m.columns.1.y) * dsy
            out.dcx = Double(m.columns.2.x) * dsx
            out.dcy = Double(m.columns.2.y) * dsy
        }
        feeder.offer(out)
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
    /// When set, this capture REPLACES what earlier captures in the project
    /// recorded wherever the camera goes. The label names it for the user
    /// ("Kitchen"); the volume is observed, not predicted.
    var rescanLabel: String?

    /// Axis-aligned box the camera moved through, accumulated live. Only
    /// used for a rescan, but tracked whenever the pose is valid because it
    /// costs two comparisons per poll.
    private var walkedMin: SIMD3<Double>?
    private var walkedMax: SIMD3<Double>?
    var isRescan: Bool { rescanLabel != nil }
    private(set) var guidance = "Preparing…"
    private(set) var framesSeen: UInt32 = 0
    /// Set when the tracker is persistently slower than the camera, so frames
    /// are being dropped before it ever sees them. Worth telling the user
    /// about because the answer is theirs — slow down, or let the phone cool
    /// — and because it degrades quietly otherwise: fewer tracked frames is
    /// weaker live pose, which is worse guidance and a worse storage gate,
    /// none of which announces itself.
    private(set) var trackerNote: String?
    private(set) var framesStored: UInt32 = 0
    private(set) var megabytesWritten: Double = 0
    private(set) var storageNote: String?
    /// How the storage note should read. Too thin and too full are opposite
    /// problems and must not look alike: one says keep going, the other says
    /// stop. Derived here rather than in the view, which used to infer it
    /// from the frame count and so painted "photos could not be saved" as
    /// ordinary grey progress whenever it happened early in a capture.
    private(set) var storageNoteLevel: StorageNoteLevel = .progress

    enum StorageNoteLevel {
        case progress   // ordinary, nobody is doing anything wrong
        case warning    // act soon
        case critical   // act now, or something has already been lost

        var tint: Color {
            switch self {
            case .progress: return .secondary
            case .warning: return .orange
            case .critical: return .red
            }
        }
    }
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

    /// Shows the route card over a running capture.
    ///
    /// Raised when the detailed scan starts and lowered by the user or by
    /// `routeCardSeconds`. Deliberately NOT a modal step before the camera
    /// starts: the flow is three things to do while walking, and a wall of
    /// instructions read before picking the phone up is forgotten by the
    /// second one. The capture runs underneath it the whole time.
    private(set) var showsRouteCard = false
    private var routeCardTask: Task<Void, Never>?
    static let routeCardSeconds: UInt64 = 14

    func presentRouteCard(auto: Bool = true) {
        routeCardTask?.cancel()
        showsRouteCard = true
        guard auto else { return }
        routeCardTask = Task { [weak self] in
            try? await Task.sleep(for: .seconds(Self.routeCardSeconds))
            guard !Task.isCancelled else { return }
            self?.showsRouteCard = false
        }
    }

    func dismissRouteCard() {
        routeCardTask?.cancel()
        routeCardTask = nil
        showsRouteCard = false
    }

    /// How the capture went, frozen at stop.
    ///
    /// Shown while the user is STILL IN THE ROOM, which is the entire point:
    /// a thin or badly-tracked capture is cheap to redo now and expensive to
    /// redo after driving home. The old end screen said "Saved:
    /// session_20260814-142230_a3f2c1" and offered a Done button, which
    /// tells someone nothing they can act on.
    struct CaptureSummary: Equatable {
        let frames: UInt32
        let keyframes: UInt32
        let readiness: Float
        let hitCap: Bool
        let wasRescan: Bool

        enum Verdict { case good, thin, full }

        var verdict: Verdict {
            if hitCap { return .full }
            return frames >= UInt32(FrameFeedContext.storedFrameTarget)
                ? .good : .thin
        }

        var headline: String {
            switch verdict {
            case .good: return wasRescan ? "Room redone" : "Room captured"
            case .thin: return "Thin capture"
            case .full: return "Capture full"
            }
        }

        var detail: String {
            switch verdict {
            case .good:
                return "\(frames) frames. Enough to reconstruct this room."
            case .thin:
                // The one case worth interrupting for: it looks fine and is
                // not, and the fix costs a minute now versus a return trip.
                return "\(frames) frames — under the \(FrameFeedContext.storedFrameTarget) "
                     + "a room usually needs. Walking it again now is much "
                     + "cheaper than coming back."
            case .full:
                return "\(frames) frames, the per-capture limit. If the room "
                     + "is not finished, capture the rest as another room in "
                     + "this project."
            }
        }
    }

    private(set) var summary: CaptureSummary?

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
                osVersion: UIDevice.current.systemVersion,
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
        if plan == .captureOnly { presentRouteCard() }
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

            // And let the tracker finish what it accepted, so the scaffold
            // the capture pass loads includes the end of the circuit — which
            // is exactly where the capture pass has to relocalize.
            await context.feeder.finish()

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
            presentRouteCard()
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
                // The storage gate needs both of these on the capture queue:
                // where the camera is, and how far it has to move before
                // another frame is worth keeping — which the engine works out
                // from how far away the scene is.
                context.publishPose(self.viewer,
                                    storeSpacingM: Double(status.store_spacing_m))
                if let centre = self.viewer?.center { self.extendWalked(centre) }

                // Drain the engine's storage directives. Polling consumes
                // them, so ignoring them loses them — which is how
                // keyframe_ids came to be empty in every session ever
                // captured on a device. The keyframe decision is made after
                // the frame is written (the tracker has to see it first), so
                // it lands in session.json at finalize rather than in
                // meta.json.
                let keyframed = CoreEngine.directives(in: status)
                    .filter { $0.is_keyframe != 0 }
                    .map(\.frame_id)
                if !keyframed.isEmpty {
                    await context.store.addKeyframeIds(keyframed)
                }
                self.recovery = RecoveryHint(status: status)
                self.framesSeen = status.frames_fed
                // The tracker's share of the camera. Some dropping is normal
                // and harmless — a relocalization sweep costs several frames
                // and recovers — so this reports a SUSTAINED shortfall, over
                // the whole session, and only once there is enough of a
                // session for the fraction to mean anything.
                let counts = context.feeder.counts
                self.trackerNote = counts.delivered >= 150
                    && counts.dropFraction > 0.35
                    ? String(format: "Tracking is behind the camera (%.0f%% of "
                             + "frames skipped) — move slower",
                             counts.dropFraction * 100)
                    : nil
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
                let failed = context.failedWrites
                if failed > 0 {
                    // Above everything else, including "nearly full". A frame
                    // that did not reach disk is gone — the ring buffer moved
                    // on — so this is the one storage message the user can
                    // still act on, by re-walking what they just covered.
                    self.storageNote = failed == 1
                        ? "1 photo could not be saved — check free space"
                        : "\(failed) photos could not be saved — check free space"
                    self.storageNoteLevel = .critical
                } else if free < FrameFeedContext.minFreeBytes {
                    // Disk beats the frame count: a capture that dies on a
                    // write halfway through a room loses the room.
                    self.storageNote = String(
                        format: "iPhone almost full (%.1f GB left) — stop soon",
                        Double(free) / 1_073_741_824)
                    self.storageNoteLevel = .critical
                } else if stored >= FrameFeedContext.storedFrameCap {
                    self.storageNote =
                        "Capture full — stop here, then continue this project "
                        + "in a new capture"
                    self.storageNoteLevel = .critical
                } else if stored >= FrameFeedContext.storedFrameWarn {
                    self.storageNote = "Nearly full: \(stored)/"
                        + "\(FrameFeedContext.storedFrameCap) — finish up"
                    self.storageNoteLevel = .warning
                } else if stored < FrameFeedContext.storedFrameTarget {
                    // The under-covered case is the one people actually hit,
                    // and it is invisible without being told: the capture
                    // looks fine, and the thinness only shows up as holes in
                    // the trained splat hours later.
                    self.storageNote = "\(stored)/"
                        + "\(FrameFeedContext.storedFrameTarget) frames — keep "
                        + "going for full coverage"
                    self.storageNoteLevel = .progress
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

    private func extendWalked(_ centre: SIMD3<Double>) {
        walkedMin = walkedMin.map { SIMD3(Swift.min($0.x, centre.x),
                                          Swift.min($0.y, centre.y),
                                          Swift.min($0.z, centre.z)) } ?? centre
        walkedMax = walkedMax.map { SIMD3(Swift.max($0.x, centre.x),
                                          Swift.max($0.y, centre.y),
                                          Swift.max($0.z, centre.z)) } ?? centre
    }

    /// The volume this capture will supersede: the walked box, grown by the
    /// distance the camera can actually see.
    ///
    /// Grown rather than used raw because the camera stands in the middle of
    /// a room and looks outwards — the box of where the FEET went is much
    /// smaller than the space that was re-covered, and superseding only the
    /// middle would leave the old walls in place alongside the new ones. The
    /// margin is deliberately modest: over-reaching into the next room
    /// discards work that was not redone, which is the worse error.
    private static let rescanMarginM = 1.5

    var rescanVolume: (min: SIMD3<Double>, max: SIMD3<Double>)? {
        guard let low = walkedMin, let high = walkedMax else { return nil }
        let margin = SIMD3<Double>(repeating: Self.rescanMarginM)
        return (low - margin, high + margin)
    }

    func refreshSnapshot() {
        snapshot = CoreEngine.shared.snapshot()
        mapRenderer.update(with: snapshot)
    }

    func renameRegion(id: UInt32, name: String) {
        _ = CoreEngine.shared.renameRegion(id: id, name: name)
        // The engine's copy is live-layer and disposable; session.json is
        // what survives the session, and is where FORMATS.md says the user's
        // names live.
        if let store = context?.store {
            Task { await store.setRegionName(id: id, name: name) }
        }
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

    /// Go round the room again after a thin capture.
    ///
    /// This starts a NEW capture rather than reopening the finished one: RAW
    /// is write-once, and a session that has been finalized is a closed
    /// record. The new capture joins the same project, so it inherits the
    /// world frame and simply adds the coverage the first pass missed —
    /// which is exactly what "walk it again" should mean. It is not a
    /// rescan: nothing is being replaced, the first pass was just short.
    func restartForMoreCoverage() {
        guard case .finished = state else { return }
        continuingProject = ProjectStore.load().first {
            $0.captures.contains { $0.directory == lastSessionDirectory }
        } ?? continuingProject
        rescanLabel = nil
        summary = nil
        context = nil
        // Straight back into capture rather than via .idle, which would
        // re-show the one-room/several-rooms chooser — a question the user
        // has already answered and cannot usefully answer differently for
        // the same room.
        start(plan: .captureOnly)
    }

    /// Directory of the capture just finished, so a follow-up can find the
    /// project it belongs to.
    private(set) var lastSessionDirectory: URL?

    func stop() {
        guard state == .capturing || state == .scouting else { return }
        state = .stopping
        pollTask?.cancel()
        manager.onFrame = nil
        manager.stop()

        Task {
            // Let in-flight encodes drain before finalizing, and the tracker
            // finish the frames it accepted — the last of them cover wherever
            // the user stopped walking.
            try? await Task.sleep(for: .milliseconds(600))
            await context?.feeder.finish()
            CoreEngine.shared.liveEnd()
            if let context {
                // Record the rescan volume BEFORE finalize, which is what
                // writes session.json out.
                if let label = rescanLabel, let volume = rescanVolume {
                    await context.store.setSupersededVolume(
                        min: (volume.min.x, volume.min.y, volume.min.z),
                        max: (volume.max.x, volume.max.y, volume.max.z),
                        label: label)
                } else if rescanLabel != nil {
                    // A rescan that never tracked has no volume, and writing
                    // an empty or guessed one would either do nothing or
                    // discard the wrong frames. The capture is still kept —
                    // it simply adds to the project instead of replacing.
                    guidance = "Could not track this rescan — kept as an "
                        + "extra capture instead of a replacement"
                }
                let locks = manager.lockState
                try? await context.store.finalize(
                    locks: (focus: locks.focus, exposure: locks.exposure,
                            whiteBalance: locks.whiteBalance))
                let stored = await context.store.storedFrames
                summary = CaptureSummary(
                    frames: stored,
                    keyframes: CoreEngine.shared.livePollStatus().keyframes,
                    readiness: readinessOverall,
                    hitCap: stored >= UInt32(FrameFeedContext.storedFrameCap),
                    wasRescan: rescanLabel != nil)
                lastSessionDirectory = context.store.directory
                let name = context.store.directory.lastPathComponent
                state = .finished(sessionName: name)
            } else {
                state = .idle
            }
        }
    }
}

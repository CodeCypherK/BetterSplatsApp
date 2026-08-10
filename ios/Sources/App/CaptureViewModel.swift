import AVFoundation
import Foundation
import Observation

/// Everything the capture-queue frame handler needs, detached from the
/// MainActor view model. Built once per session; @unchecked Sendable because
/// mutable state is guarded by `lock` and the rest is immutable or actors.
final class FrameFeedContext: @unchecked Sendable {
    /// Hard cap on stored frames (~1.3 MB each ≈ 1.2 GB); UI warns earlier.
    static let storedFrameCap = 900
    static let storedFrameWarn = 700

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

        // Adaptive storage gate (M1: cadence + exposure sanity; the engine's
        // motion-aware store directives take over in M4). Raw storage is
        // never dropped for backpressure — the gate only thins cadence when
        // the encoder can't keep up, and that surfaces in the UI meter.
        lock.lock()
        let elapsed = frame.tCapture - lastStoreTime
        let busy = encodesInFlight >= 2
        let capped = storeCommits >= Self.storedFrameCap
        let shouldStore = !busy && !capped && elapsed >= 0.30
            && stats.overexposedFraction < 0.10
        if shouldStore {
            lastStoreTime = frame.tCapture
            encodesInFlight += 1
            storeCommits += 1
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
            storeReason: "gate")

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
        case capturing
        case stopping
        case finished(sessionName: String)
        case failed(String)
    }

    private(set) var state: State = .idle
    private(set) var guidance = "Preparing…"
    private(set) var framesSeen: UInt32 = 0
    private(set) var framesStored: UInt32 = 0
    private(set) var megabytesWritten: Double = 0
    private(set) var storageNote: String?
    private(set) var readinessOverall: Float = 0
    private(set) var snapshot = CoreEngine.Snapshot()

    private let manager = CaptureManager()
    private var context: FrameFeedContext?
    private var pollTask: Task<Void, Never>?
    private var snapshotTick = 0

    let previewRenderer = VideoPreviewRenderer()
    let mapRenderer = MapRenderer()

    var isCapturing: Bool { state == .capturing }

    func start() {
        switch state {
        case .idle, .finished, .failed:
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
                fps: 30)
        } catch {
            state = .failed("Could not create session storage: \(error)")
            return
        }

        let engine = CoreEngine.shared
        guard engine.liveBegin(sessionDir: store.directory.path) == BS_OK else {
            state = .failed("Engine rejected session: \(engine.lastError)")
            return
        }

        let context = FrameFeedContext(
            store: store, previewRenderer: previewRenderer,
            videoDims: manager.videoDimensions)
        self.context = context
        manager.onFrame = { frame in
            context.handle(frame)  // capture queue
        }
        manager.start()
        state = .capturing
        guidance = "Move slowly and keep the scene in view"
        startPolling()
    }

    private func startPolling() {
        pollTask?.cancel()
        pollTask = Task { [weak self] in
            while !Task.isCancelled {
                try? await Task.sleep(for: .milliseconds(100))
                guard let self, self.isCapturing, let context = self.context
                else { continue }
                let status = CoreEngine.shared.livePollStatus()
                self.framesSeen = status.frames_fed
                self.guidance = Self.guidanceText(for: status)
                self.readinessOverall = status.readiness_overall
                self.framesStored = await context.store.storedFrames
                self.megabytesWritten =
                    Double(await context.store.storedBytes) / 1_048_576.0
                let stored = Int(self.framesStored)
                if stored >= FrameFeedContext.storedFrameCap {
                    self.storageNote =
                        "Storage limit reached — stop and reconstruct"
                } else if stored >= FrameFeedContext.storedFrameWarn {
                    self.storageNote = "Storage: \(stored)/"
                        + "\(FrameFeedContext.storedFrameCap) frames"
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

    func stop() {
        guard state == .capturing else { return }
        state = .stopping
        pollTask?.cancel()
        manager.onFrame = nil
        manager.stop()

        Task {
            // Let in-flight encodes drain before finalizing.
            try? await Task.sleep(for: .milliseconds(600))
            CoreEngine.shared.liveEnd()
            if let context {
                try? await context.store.finalize()
                let name = context.store.directory.lastPathComponent
                state = .finished(sessionName: name)
                guidance = "Saved \(name)"
            } else {
                state = .idle
            }
        }
    }
}

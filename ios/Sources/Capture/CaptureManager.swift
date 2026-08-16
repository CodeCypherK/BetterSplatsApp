import AVFoundation
import CoreMotion
import UIKit

/// Owns the AVFoundation LiDAR capture pipeline: synchronized 1920x1440
/// 420f video + 320x240 Float16 depth at 30 fps.
///
/// Configuration is correctness-critical for reconstruction (see
/// docs/ARCHITECTURE.md): geometric distortion correction OFF (we model the
/// lens ourselves from the calibration LUT), stabilization OFF (warping
/// invalidates intrinsics), depth temporal filtering OFF (raw measurements
/// only), and focus + white balance + ISO locked after the scene settles —
/// shutter duration stays free so a walk from a bright window into a
/// hallway does not clip or crush. Focus still has to freeze: one camera
/// model for the whole session. WHEN they lock is the caller's decision:
/// see `settleThenLock()`.
final class CaptureManager: NSObject {
    enum CaptureError: LocalizedError {
        case noLiDARDevice
        case noCompatibleFormat
        case configurationFailed(String)

        var errorDescription: String? {
            switch self {
            case .noLiDARDevice:
                return "This device has no LiDAR camera. A Pro model iPhone (12 Pro or later) is required."
            case .noCompatibleFormat:
                return "No compatible synchronized video+depth format found."
            case .configurationFailed(let why):
                return "Camera configuration failed: \(why)"
            }
        }
    }

    struct Configuration {
        var videoWidth = 1920
        var videoHeight = 1440
        var frameRate = 30
    }

    /// Called on the capture queue for every synchronized frame.
    var onFrame: ((CapturedFrame) -> Void)?

    private let session = AVCaptureSession()
    private let videoOutput = AVCaptureVideoDataOutput()
    private let depthOutput = AVCaptureDepthDataOutput()
    private var synchronizer: AVCaptureDataOutputSynchronizer?
    private var device: AVCaptureDevice?
    private let captureQueue = DispatchQueue(label: "bs.capture", qos: .userInteractive)
    private let motion = CMMotionManager()
    private var lastFrameTime: Double?
    private var gyroAccum = SIMD3<Double>(0, 0, 0)
    private var lastGyroTime: Double?
    private let gyroLock = NSLock()

    private(set) var videoDimensions: (width: Int, height: Int) = (0, 0)
    private(set) var depthDimensions: (width: Int, height: Int) = (0, 0)

    /// What actually got locked once the settle delay elapsed. Read after
    /// capture starts; recorded into session.json so the reconstruction side
    /// knows whether photometric consistency can be assumed.
    struct LockState {
        var focus = false
        /// Full AE lock (ISO and shutter). We no longer ask for this.
        var exposure = false
        var iso = false
        var whiteBalance = false
    }
    private let lockStateLock = NSLock()
    private var lockState_ = LockState()
    private(set) var lockState: LockState {
        get { lockStateLock.lock(); defer { lockStateLock.unlock() }; return lockState_ }
        set { lockStateLock.lock(); lockState_ = newValue; lockStateLock.unlock() }
    }

    static var hasLiDAR: Bool {
        AVCaptureDevice.default(.builtInLiDARDepthCamera, for: .video, position: .back) != nil
    }

    func configure(_ config: Configuration = Configuration()) throws {
        guard let device = AVCaptureDevice.default(
            .builtInLiDARDepthCamera, for: .video, position: .back
        ) else {
            throw CaptureError.noLiDARDevice
        }
        self.device = device

        // Pick a 4:3 420f video format at the requested size that supports
        // Float16 depth delivery.
        var chosenFormat: AVCaptureDevice.Format?
        var chosenDepthFormat: AVCaptureDevice.Format?
        for format in device.formats {
            let dims = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
            let subtype = CMFormatDescriptionGetMediaSubType(format.formatDescription)
            guard Int(dims.width) == config.videoWidth,
                  Int(dims.height) == config.videoHeight,
                  subtype == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
            else { continue }

            let depthFormats = format.supportedDepthDataFormats.filter {
                CMFormatDescriptionGetMediaSubType($0.formatDescription)
                    == kCVPixelFormatType_DepthFloat16
            }
            guard let bestDepth = depthFormats.max(by: {
                CMVideoFormatDescriptionGetDimensions($0.formatDescription).width
                    < CMVideoFormatDescriptionGetDimensions($1.formatDescription).width
            }) else { continue }

            chosenFormat = format
            chosenDepthFormat = bestDepth
            break
        }
        guard let format = chosenFormat, let depthFormat = chosenDepthFormat else {
            throw CaptureError.noCompatibleFormat
        }

        session.beginConfiguration()
        defer { session.commitConfiguration() }
        session.sessionPreset = .inputPriority

        do {
            try device.lockForConfiguration()
            device.activeFormat = format
            device.activeDepthDataFormat = depthFormat
            let duration = CMTime(value: 1, timescale: CMTimeScale(config.frameRate))
            device.activeVideoMinFrameDuration = duration
            device.activeVideoMaxFrameDuration = duration
            if device.isGeometricDistortionCorrectionSupported {
                device.isGeometricDistortionCorrectionEnabled = false
            }
            device.unlockForConfiguration()
        } catch {
            throw CaptureError.configurationFailed("device lock: \(error.localizedDescription)")
        }

        let input: AVCaptureDeviceInput
        do {
            input = try AVCaptureDeviceInput(device: device)
        } catch {
            throw CaptureError.configurationFailed("input: \(error.localizedDescription)")
        }
        guard session.canAddInput(input) else {
            throw CaptureError.configurationFailed("cannot add camera input")
        }
        session.addInput(input)

        videoOutput.videoSettings = [
            kCVPixelBufferPixelFormatTypeKey as String:
                kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
        ]
        videoOutput.alwaysDiscardsLateVideoFrames = true
        guard session.canAddOutput(videoOutput) else {
            throw CaptureError.configurationFailed("cannot add video output")
        }
        session.addOutput(videoOutput)

        depthOutput.isFilteringEnabled = false
        depthOutput.alwaysDiscardsLateDepthData = true
        guard session.canAddOutput(depthOutput) else {
            throw CaptureError.configurationFailed("cannot add depth output")
        }
        session.addOutput(depthOutput)

        if let connection = videoOutput.connection(with: .video),
           connection.isVideoStabilizationSupported {
            connection.preferredVideoStabilizationMode = .off
        }

        let dims = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
        videoDimensions = (Int(dims.width), Int(dims.height))
        let ddims = CMVideoFormatDescriptionGetDimensions(depthFormat.formatDescription)
        depthDimensions = (Int(ddims.width), Int(ddims.height))

        let sync = AVCaptureDataOutputSynchronizer(dataOutputs: [videoOutput, depthOutput])
        sync.setDelegate(self, queue: captureQueue)
        synchronizer = sync
    }

    func start() {
        startGyro()
        captureQueue.async { [session] in
            session.startRunning()
        }
        // NOTE: nothing is locked here. See settleThenLock() — locking on a
        // timer from start() aims the whole session at whatever the phone
        // happened to be pointing at in its first second and a half.
    }

    /// Converges AF/AE/AWB on what the camera is looking at NOW, then freezes
    /// focus, white balance and ISO. Shutter stays in custom mode and is
    /// steered from frame luma so brightness can still track the room.
    ///
    /// The caller decides when the view is representative, and that decision
    /// cannot be made in here. This used to fire on a 1.5 s timer from
    /// `start()`, which is wrong in the exact case the app now creates on
    /// purpose: the floor calibration asks the user to point the phone at the
    /// floor and step forward, so the timer locked focus at about a metre,
    /// exposure for a patch of floor, and white balance to the floor's
    /// colour — and none of the three ever re-adjusts, by design. Everything
    /// past the first couple of metres would be soft for the whole session,
    /// with nothing in the UI to say so.
    ///
    /// The one-shot `.autoFocus` / `.autoExpose` / `.autoWhiteBalance` modes
    /// do the work: each performs a single convergence on the current scene
    /// and then reverts to `.locked` by itself. Polling `isAdjusting*` is
    /// only to find out when that has happened, so what gets written into
    /// session.json is what the hardware actually did rather than what was
    /// asked of it.
    func settleThenLock() {
        captureQueue.async { [weak self] in
            guard let self, let device = self.device else { return }
            do {
                try device.lockForConfiguration()
                defer { device.unlockForConfiguration() }
                // Centre-weighted: the middle of the frame is the surface the
                // user is walking toward, which is what the session should be
                // exposed and focused for.
                if device.isFocusPointOfInterestSupported {
                    device.focusPointOfInterest = CGPoint(x: 0.5, y: 0.5)
                }
                if device.isExposurePointOfInterestSupported {
                    device.exposurePointOfInterest = CGPoint(x: 0.5, y: 0.5)
                }
                if device.isFocusModeSupported(.autoFocus) {
                    device.focusMode = .autoFocus
                }
                // Don't run AE if the user already pinned ISO — autoExpose
                // would move it before lockCaptureSettings can restore it.
                if !self.isoChosenByUser, device.isExposureModeSupported(.autoExpose) {
                    device.exposureMode = .autoExpose
                }
                if device.isWhiteBalanceModeSupported(.autoWhiteBalance) {
                    device.whiteBalanceMode = .autoWhiteBalance
                }
            } catch {
                // Could not even ask. Fall through to the forced lock below,
                // which reports honestly whatever it manages.
            }
            self.awaitConvergence(attemptsLeft: Self.convergenceAttempts)
        }
    }

    /// ~4 s of 100 ms polls. The deadline is a backstop, not the mechanism:
    /// a scene with nothing to focus on (a blank wall in dim light) can hunt
    /// indefinitely, and a session that never locks is worse than one locked
    /// on an imperfect guess — at least the imperfect guess is consistent
    /// across every frame, which is what the splat actually needs.
    private static let convergenceAttempts = 40

    private func awaitConvergence(attemptsLeft: Int) {
        guard let device else { return }
        let exposureBusy = !isoChosenByUser && device.isAdjustingExposure
        let settled = !device.isAdjustingFocus && !exposureBusy
            && !device.isAdjustingWhiteBalance
        guard settled || attemptsLeft <= 0 else {
            captureQueue.asyncAfter(deadline: .now() + 0.1) { [weak self] in
                self?.awaitConvergence(attemptsLeft: attemptsLeft - 1)
            }
            return
        }
        lockCaptureSettings()
    }

    func stop() {
        motion.stopGyroUpdates()
        captureQueue.async { [weak self] in
            guard let self else { return }
            self.lockedISO = nil
            self.isoChosenByUser = false
            if self.session.isRunning { self.session.stopRunning() }
        }
    }

    /// ISO frozen at whatever AE settled on, unless the user set it; shutter
    /// is adjusted from luma.
    /// Nil until `lockCaptureSettings` or `setISO` runs.
    private var lockedISO: Float?
    private var isoChosenByUser = false
    private var lastShutterSteer: Double = 0

    var isoLimits: (min: Float, max: Float) {
        guard let format = device?.activeFormat else { return (50, 1600) }
        return (format.minISO, format.maxISO)
    }

    var currentISO: Float {
        if let locked = lockedISO { return locked }
        return device?.iso ?? isoLimits.min
    }

    /// Pin ISO to `iso` and switch to custom exposure so shutter can still
    /// move. Safe to call before or after the photometric settle.
    func setISO(_ iso: Float) {
        captureQueue.async { [weak self] in
            self?.applyISO(iso, chosenByUser: true)
        }
    }

    private func applyISO(_ iso: Float, chosenByUser: Bool) {
        guard let device else { return }
        let limits = isoLimits
        let clamped = min(limits.max, max(limits.min, iso))
        do {
            try device.lockForConfiguration()
            defer { device.unlockForConfiguration() }
            if device.isExposureModeSupported(.custom) {
                let duration = device.exposureDuration
                device.setExposureModeCustom(duration: duration, iso: clamped,
                                             completionHandler: nil)
                lockedISO = clamped
                if chosenByUser { isoChosenByUser = true }
            }
        } catch {
            // Leave the previous ISO in place.
        }
    }

    /// Freezes focus, white balance and ISO on whatever they converged to.
    /// Shutter is left adjustable: a locked ISO keeps sensor gain (and
    /// therefore noise) consistent, which is the photometric axis a splat
    /// actually cares about, while duration can still follow a window-to-
    /// hallway lighting change. AVFoundation has no ISO-priority AE, so
    /// shutter is driven from the same luma stats the storage gate already
    /// computes — `steerShutter`.
    ///
    /// Focus still locks: one lens position, one camera model.
    private func lockCaptureSettings() {
        guard let device else { return }
        var state = LockState()
        do {
            try device.lockForConfiguration()
            defer { device.unlockForConfiguration() }
            if device.isFocusModeSupported(.locked) {
                device.focusMode = .locked
            }
            if device.isWhiteBalanceModeSupported(.locked) {
                device.whiteBalanceMode = .locked
            }
            if device.isExposureModeSupported(.custom) {
                let iso: Float
                if isoChosenByUser, let chosen = lockedISO {
                    iso = chosen
                } else {
                    iso = min(device.activeFormat.maxISO,
                              max(device.activeFormat.minISO, device.iso))
                }
                device.setExposureModeCustom(duration: device.exposureDuration,
                                             iso: iso,
                                             completionHandler: nil)
                lockedISO = iso
            } else if device.isExposureModeSupported(.locked) {
                // No custom mode: fall back to a full AE lock rather than
                // leaving ISO free to hunt for the rest of the session.
                device.exposureMode = .locked
            }
        } catch {
            // Settings stay continuous; per-frame ISO and duration are still
            // recorded, so the solve can compensate after the fact.
        }
        state.focus = device.focusMode == .locked
        state.exposure = device.exposureMode == .locked
        state.iso = lockedISO != nil && device.exposureMode == .custom
        state.whiteBalance = device.whiteBalanceMode == .locked
        lockState = state
    }

    /// Nudge shutter only, holding the ISO captured at lock. Called from the
    /// capture queue with the frame's luma stats. No-ops until ISO is frozen
    /// and skips tiny steps so we are not reconfiguring the device at 30 Hz.
    func steerShutter(_ stats: FrameAnalysis.LumaStats) {
        guard let device, let iso = lockedISO,
              device.exposureMode == .custom,
              !device.isAdjustingExposure
        else { return }
        let t = ProcessInfo.processInfo.systemUptime
        guard t - lastShutterSteer >= 0.15 else { return }

        let minS = device.activeFormat.minExposureDuration.seconds
        let frameS = device.activeVideoMaxFrameDuration.seconds
        let formatMax = device.activeFormat.maxExposureDuration.seconds
        let maxS = min(formatMax, frameS > 0 ? frameS : formatMax)
        var seconds = device.exposureDuration.seconds
        // Target a mid-grey luma. Overexposure wins: clipped highlights are
        // gone, a slightly dark frame is not.
        if stats.overexposedFraction > 0.03 {
            seconds *= 0.72
        } else if stats.meanLuma > 1 {
            let ratio = 110.0 / stats.meanLuma
            seconds *= min(1.22, max(0.82, ratio))
        }
        seconds = min(maxS, max(minS, seconds))
        let current = device.exposureDuration.seconds
        guard current > 0, abs(seconds - current) / current > 0.06 else { return }

        do {
            try device.lockForConfiguration()
            defer { device.unlockForConfiguration() }
            let duration = CMTime(seconds: seconds, preferredTimescale: 1_000_000)
            device.setExposureModeCustom(duration: duration, iso: iso,
                                         completionHandler: nil)
            lastShutterSteer = t
        } catch {
            // Leave the last duration in place.
        }
    }

    private func startGyro() {
        guard motion.isGyroAvailable else { return }
        motion.gyroUpdateInterval = 1.0 / 100.0
        motion.startGyroUpdates(to: OperationQueue()) { [weak self] data, _ in
            guard let self, let data else { return }
            self.gyroLock.lock()
            defer { self.gyroLock.unlock() }
            let t = data.timestamp
            if let last = self.lastGyroTime {
                let dt = t - last
                self.gyroAccum.x += data.rotationRate.x * dt
                self.gyroAccum.y += data.rotationRate.y * dt
                self.gyroAccum.z += data.rotationRate.z * dt
            }
            self.lastGyroTime = t
        }
    }

    private func takeGyroDelta() -> (x: Float, y: Float, z: Float)? {
        guard motion.isGyroAvailable else { return nil }
        gyroLock.lock()
        defer {
            gyroAccum = .zero
            gyroLock.unlock()
        }
        return (Float(gyroAccum.x), Float(gyroAccum.y), Float(gyroAccum.z))
    }
}

extension CaptureManager: AVCaptureDataOutputSynchronizerDelegate {
    func dataOutputSynchronizer(
        _ synchronizer: AVCaptureDataOutputSynchronizer,
        didOutput collection: AVCaptureSynchronizedDataCollection
    ) {
        guard
            let videoData = collection.synchronizedData(for: videoOutput)
                as? AVCaptureSynchronizedSampleBufferData,
            !videoData.sampleBufferWasDropped,
            let pixelBuffer = CMSampleBufferGetImageBuffer(videoData.sampleBuffer),
            let depthSync = collection.synchronizedData(for: depthOutput)
                as? AVCaptureSynchronizedDepthData,
            !depthSync.depthDataWasDropped
        else { return }

        // Normalize depth to Float16 regardless of the delivered type.
        let depthData: AVDepthData
        if depthSync.depthData.depthDataType == kCVPixelFormatType_DepthFloat16 {
            depthData = depthSync.depthData
        } else {
            depthData = depthSync.depthData.converting(
                toDepthDataType: kCVPixelFormatType_DepthFloat16)
        }

        let tCapture = CMSampleBufferGetPresentationTimeStamp(videoData.sampleBuffer)
            .seconds
        let frame = CapturedFrame(
            pixelBuffer: pixelBuffer,
            depthData: depthData,
            tCapture: tCapture,
            tDepth: depthSync.timestamp.seconds,
            calibration: depthData.cameraCalibrationData,
            exposureDuration: device.map { $0.exposureDuration.seconds } ?? 0,
            iso: device.map { Double($0.iso) } ?? 0,
            exposureBias: device.map { Double($0.exposureTargetBias) } ?? 0,
            gyroDelta: takeGyroDelta()
        )
        onFrame?(frame)
    }
}

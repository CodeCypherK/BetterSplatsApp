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
/// only), and autofocus locked shortly after start so one camera model
/// serves the whole session.
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
        captureQueue.async { [session, weak self] in
            session.startRunning()
            // Run continuous AF briefly to acquire focus, then lock it for
            // the rest of the session (single camera model).
            self?.captureQueue.asyncAfter(deadline: .now() + 1.5) {
                self?.lockFocus()
            }
        }
    }

    func stop() {
        motion.stopGyroUpdates()
        captureQueue.async { [session] in
            if session.isRunning { session.stopRunning() }
        }
    }

    private func lockFocus() {
        guard let device, device.isFocusModeSupported(.locked) else { return }
        do {
            try device.lockForConfiguration()
            device.focusMode = .locked
            device.unlockForConfiguration()
        } catch {
            // Focus stays continuous; per-frame intrinsics still recorded.
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

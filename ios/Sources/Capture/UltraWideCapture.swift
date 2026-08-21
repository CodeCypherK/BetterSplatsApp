import AVFoundation
import CoreMedia
import CoreVideo
import Foundation
import UIKit

/// Which exposure axis the user has frozen. The other axis is steered from
/// frame luma so brightness can still track the room.
enum ExposureAxisLock: Equatable {
    case none
    case iso
    case shutter
}

/// True 0.5× rear ultra-wide via AVCapture at max video resolution.
///
/// Session configuration and the preview layer run on the **main** thread.
/// Only `startRunning` / frame callbacks use the capture queue — creating
/// `AVCaptureVideoPreviewLayer` off-main was crashing on Start.
final class UltraWideCapture: NSObject {
    enum CaptureError: LocalizedError {
        case noUltraWide
        case notAuthorized
        case configurationFailed(String)
        var errorDescription: String? {
            switch self {
            case .noUltraWide:
                return "No ultra-wide (0.5×) camera on this device"
            case .notAuthorized:
                return "Camera access is not granted"
            case .configurationFailed(let s):
                return s
            }
        }
    }

    static var isAvailable: Bool {
        AVCaptureDevice.default(.builtInUltraWideCamera, for: .video,
                                position: .back) != nil
            || AVCaptureDevice.default(.builtInDualWideCamera, for: .video,
                                       position: .back) != nil
    }

    private let session = AVCaptureSession()
    private let videoOut = AVCaptureVideoDataOutput()
    private let queue = DispatchQueue(label: "bs.ultrawide", qos: .userInitiated)
    private var device: AVCaptureDevice?
    private(set) var previewLayer: AVCaptureVideoPreviewLayer?
    private(set) var dimensions: (width: Int, height: Int) = (1920, 1440)

    private let exposureLock = NSLock()
    private var axisLock: ExposureAxisLock = .none
    private var frozenISO: Float?
    private var frozenShutter: CMTime?
    private var lastSteerUptime: Double = 0

    var onFrame: ((CVPixelBuffer, CMTime) -> Void)?

    var exposureAxisLock: ExposureAxisLock {
        exposureLock.lock(); defer { exposureLock.unlock() }
        return axisLock
    }

    var lockedISOValue: Float? {
        exposureLock.lock(); defer { exposureLock.unlock() }
        return frozenISO
    }

    var lockedShutterSeconds: Double? {
        exposureLock.lock(); defer { exposureLock.unlock() }
        guard let t = frozenShutter else { return nil }
        return CMTimeGetSeconds(t)
    }

    /// Call from the main actor only.
    func start() throws {
        assert(Thread.isMainThread)

        switch AVCaptureDevice.authorizationStatus(for: .video) {
        case .authorized:
            break
        case .notDetermined:
            throw CaptureError.configurationFailed(
                "Grant camera access on the home screen, then try again")
        default:
            throw CaptureError.notAuthorized
        }

        if session.isRunning {
            session.stopRunning()
        }
        clearExposureLock()

        try configureSessionOnMain()

        if previewLayer == nil {
            let preview = AVCaptureVideoPreviewLayer(session: session)
            preview.videoGravity = .resizeAspectFill
            previewLayer = preview
        } else {
            previewLayer?.session = session
        }

        queue.async { [session] in
            if !session.isRunning {
                session.startRunning()
            }
        }
    }

    func stop() {
        onFrame = nil
        queue.async { [weak self] in
            guard let self else { return }
            self.clearExposureLock()
            if self.session.isRunning { self.session.stopRunning() }
        }
    }

    func lockISO() { queue.async { [weak self] in self?.applyLockISO() } }
    func lockShutter() { queue.async { [weak self] in self?.applyLockShutter() } }
    func unlockExposure() { queue.async { [weak self] in self?.applyUnlockExposure() } }

    private func configureSessionOnMain() throws {
        session.beginConfiguration()
        defer { session.commitConfiguration() }
        session.sessionPreset = .inputPriority

        for old in session.inputs { session.removeInput(old) }
        for old in session.outputs { session.removeOutput(old) }
        device = nil

        let cam: AVCaptureDevice
        let useDualHalfZoom: Bool
        if let ultra = AVCaptureDevice.default(
            .builtInUltraWideCamera, for: .video, position: .back) {
            cam = ultra
            useDualHalfZoom = false
        } else if let dual = AVCaptureDevice.default(
            .builtInDualWideCamera, for: .video, position: .back) {
            cam = dual
            useDualHalfZoom = true
        } else {
            throw CaptureError.noUltraWide
        }

        let formats = Self.rankedFormats(cam)
        guard let format = formats.first else {
            throw CaptureError.configurationFailed("no suitable ultra-wide format")
        }

        do {
            try cam.lockForConfiguration()
            cam.activeFormat = format
            if useDualHalfZoom {
                cam.videoZoomFactor = cam.minAvailableVideoZoomFactor
            }
            // HDR video looks washed out in preview + JPEGs for this pipeline.
            if cam.activeFormat.isVideoHDRSupported {
                cam.automaticallyAdjustsVideoHDREnabled = false
                cam.isVideoHDREnabled = false
            }
            let want = CMTime(value: 1, timescale: 30)
            if let range = format.videoSupportedFrameRateRanges.first(where: {
                CMTimeCompare($0.minFrameDuration, want) <= 0
                    && CMTimeCompare(want, $0.maxFrameDuration) <= 0
            }) {
                _ = range
                cam.activeVideoMinFrameDuration = want
                cam.activeVideoMaxFrameDuration = want
            }
            let d = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
            dimensions = (Int(d.width), Int(d.height))
            cam.unlockForConfiguration()
        } catch {
            throw CaptureError.configurationFailed(
                "device lock: \(error.localizedDescription)")
        }

        let newInput: AVCaptureDeviceInput
        do {
            newInput = try AVCaptureDeviceInput(device: cam)
        } catch {
            throw CaptureError.configurationFailed(
                "input: \(error.localizedDescription)")
        }
        guard session.canAddInput(newInput) else {
            throw CaptureError.configurationFailed("cannot add ultra-wide input")
        }
        session.addInput(newInput)
        device = cam

        videoOut.alwaysDiscardsLateVideoFrames = true
        videoOut.setSampleBufferDelegate(self, queue: queue)
        guard session.canAddOutput(videoOut) else {
            throw CaptureError.configurationFailed("cannot add video output")
        }
        session.addOutput(videoOut)

        // Prefer video-range 420v — full-range 420f reads washed/pastel on many
        // UW formats when preview + CIImage assume video levels.
        let videoRange = kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange
        let fullRange = kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
        if videoOut.availableVideoPixelFormatTypes.contains(videoRange) {
            videoOut.videoSettings = [
                kCVPixelBufferPixelFormatTypeKey as String: videoRange
            ]
        } else if videoOut.availableVideoPixelFormatTypes.contains(fullRange) {
            videoOut.videoSettings = [
                kCVPixelBufferPixelFormatTypeKey as String: fullRange
            ]
        } else {
            videoOut.videoSettings = nil
        }
    }

    private static func rankedFormats(_ device: AVCaptureDevice) -> [AVCaptureDevice.Format] {
        let all = device.formats.filter { format in
            let dims = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
            let okFps = format.videoSupportedFrameRateRanges.contains {
                $0.maxFrameRate >= 29.5
            }
            return okFps && dims.width >= 1280
        }
        // Prefer max resolution among SDR formats when possible.
        let sdr = all.filter { !$0.isVideoHDRSupported }
        let pool = sdr.isEmpty ? all : sdr
        return pool.sorted { a, b in
            let da = CMVideoFormatDescriptionGetDimensions(a.formatDescription)
            let db = CMVideoFormatDescriptionGetDimensions(b.formatDescription)
            return Int(da.width) * Int(da.height) > Int(db.width) * Int(db.height)
        }
    }

    func steerExposure(stats: FrameAnalysis.LumaStats) {
        exposureLock.lock()
        let mode = axisLock
        let iso = frozenISO
        let shutter = frozenShutter
        exposureLock.unlock()
        guard mode != .none, let device, device.exposureMode == .custom,
              !device.isAdjustingExposure else { return }
        let now = ProcessInfo.processInfo.systemUptime
        guard now - lastSteerUptime >= 0.15 else { return }
        lastSteerUptime = now
        switch mode {
        case .iso:
            guard let iso else { return }
            steerShutter(device: device, iso: iso, stats: stats)
        case .shutter:
            guard let shutter else { return }
            steerISO(device: device, shutter: shutter, stats: stats)
        case .none: break
        }
    }

    private func applyLockISO() {
        guard let device, device.isExposureModeSupported(.custom) else { return }
        do {
            try device.lockForConfiguration()
            defer { device.unlockForConfiguration() }
            let iso = min(device.activeFormat.maxISO,
                          max(device.activeFormat.minISO, device.iso))
            device.setExposureModeCustom(duration: device.exposureDuration, iso: iso,
                                         completionHandler: nil)
            exposureLock.lock()
            axisLock = .iso; frozenISO = iso; frozenShutter = nil
            exposureLock.unlock()
        } catch {}
    }

    private func applyLockShutter() {
        guard let device, device.isExposureModeSupported(.custom) else { return }
        do {
            try device.lockForConfiguration()
            defer { device.unlockForConfiguration() }
            let duration = device.exposureDuration
            let iso = min(device.activeFormat.maxISO,
                          max(device.activeFormat.minISO, device.iso))
            device.setExposureModeCustom(duration: duration, iso: iso,
                                         completionHandler: nil)
            exposureLock.lock()
            axisLock = .shutter; frozenShutter = duration; frozenISO = nil
            exposureLock.unlock()
        } catch {}
    }

    private func applyUnlockExposure() {
        guard let device else { return }
        do {
            try device.lockForConfiguration()
            defer { device.unlockForConfiguration() }
            if device.isExposureModeSupported(.continuousAutoExposure) {
                device.exposureMode = .continuousAutoExposure
            }
        } catch {}
        clearExposureLock()
    }

    private func clearExposureLock() {
        exposureLock.lock()
        axisLock = .none; frozenISO = nil; frozenShutter = nil
        exposureLock.unlock()
    }

    private func steerShutter(device: AVCaptureDevice, iso: Float,
                              stats: FrameAnalysis.LumaStats) {
        let minS = device.activeFormat.minExposureDuration.seconds
        let frameS = device.activeVideoMaxFrameDuration.seconds
        let formatMax = device.activeFormat.maxExposureDuration.seconds
        let maxS = min(formatMax, frameS > 0 ? frameS : formatMax)
        var seconds = device.exposureDuration.seconds
        if stats.overexposedFraction > 0.03 { seconds *= 0.72 }
        else if stats.meanLuma > 1 {
            seconds *= min(1.22, max(0.82, 110.0 / stats.meanLuma))
        }
        seconds = min(maxS, max(minS, seconds))
        let current = device.exposureDuration.seconds
        guard current > 0, abs(seconds - current) / current > 0.06 else { return }
        do {
            try device.lockForConfiguration()
            defer { device.unlockForConfiguration() }
            device.setExposureModeCustom(
                duration: CMTime(seconds: seconds, preferredTimescale: 1_000_000),
                iso: iso, completionHandler: nil)
        } catch {}
    }

    private func steerISO(device: AVCaptureDevice, shutter: CMTime,
                          stats: FrameAnalysis.LumaStats) {
        let minISO = device.activeFormat.minISO
        let maxISO = device.activeFormat.maxISO
        var iso = device.iso
        if stats.overexposedFraction > 0.03 { iso *= 0.72 }
        else if stats.meanLuma > 1 {
            iso *= min(1.22, max(0.82, Float(110.0 / stats.meanLuma)))
        }
        iso = min(maxISO, max(minISO, iso))
        guard abs(iso - device.iso) / max(device.iso, 1) > 0.06 else { return }
        do {
            try device.lockForConfiguration()
            defer { device.unlockForConfiguration() }
            device.setExposureModeCustom(duration: shutter, iso: iso,
                                         completionHandler: nil)
        } catch {}
    }
}

extension UltraWideCapture: AVCaptureVideoDataOutputSampleBufferDelegate {
    func captureOutput(_ output: AVCaptureOutput,
                       didOutput sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) {
        guard let buf = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        let t = CMSampleBufferGetPresentationTimeStamp(sampleBuffer)
        exposureLock.lock()
        let due = axisLock != .none
            && ProcessInfo.processInfo.systemUptime - lastSteerUptime >= 0.15
        exposureLock.unlock()
        if due { steerExposure(stats: FrameAnalysis.lumaStats(of: buf)) }
        onFrame?(buf, t)
    }
}

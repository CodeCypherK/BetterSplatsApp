import AVFoundation
import CoreMedia
import CoreVideo
import Foundation
import UIKit

/// Which exposure axis the user has frozen. The other axis is steered from
/// frame luma so brightness can still track the room.
enum ExposureAxisLock: Equatable {
    case none
    /// ISO frozen; shutter duration is nudged from luma.
    case iso
    /// Shutter frozen; ISO is nudged from luma.
    case shutter
}

/// True 0.5× rear ultra-wide via AVCapture at the highest video resolution
/// the device offers for that lens.
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
    private var input: AVCaptureDeviceInput?
    private var device: AVCaptureDevice?
    private(set) var previewLayer: AVCaptureVideoPreviewLayer?
    private(set) var dimensions: (width: Int, height: Int) = (1920, 1440)

    private let exposureLock = NSLock()
    private var axisLock: ExposureAxisLock = .none
    private var frozenISO: Float?
    private var frozenShutter: CMTime?
    private var lastSteerUptime: Double = 0

    /// Called on `queue` for every frame.
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

    func start() throws {
        let auth = AVCaptureDevice.authorizationStatus(for: .video)
        if auth == .notDetermined {
            let sem = DispatchSemaphore(value: 0)
            var granted = false
            AVCaptureDevice.requestAccess(for: .video) { ok in
                granted = ok
                sem.signal()
            }
            _ = sem.wait(timeout: .now() + 15)
            if !granted { throw CaptureError.notAuthorized }
        } else if auth != .authorized {
            throw CaptureError.notAuthorized
        }

        var thrown: Error?
        queue.sync {
            do {
                try self.configureLocked()
            } catch {
                thrown = error
            }
        }
        if let thrown { throw thrown }
        queue.async { [session] in
            if !session.isRunning { session.startRunning() }
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

    /// Freeze ISO at the current value; shutter stays adjustable (steered).
    func lockISO() {
        queue.async { [weak self] in self?.applyLockISO() }
    }

    /// Freeze shutter at the current duration; ISO stays adjustable (steered).
    func lockShutter() {
        queue.async { [weak self] in self?.applyLockShutter() }
    }

    func unlockExposure() {
        queue.async { [weak self] in self?.applyUnlockExposure() }
    }

    /// Called on the capture queue with luma stats for the current frame.
    func steerExposure(stats: FrameAnalysis.LumaStats) {
        exposureLock.lock()
        let mode = axisLock
        let iso = frozenISO
        let shutter = frozenShutter
        exposureLock.unlock()

        guard mode != .none, let device, device.exposureMode == .custom,
              !device.isAdjustingExposure
        else { return }

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
        case .none:
            break
        }
    }

    private func configureLocked() throws {
        if session.isRunning { session.stopRunning() }
        clearExposureLock()

        session.beginConfiguration()
        defer { session.commitConfiguration() }

        session.sessionPreset = .inputPriority

        for old in session.inputs { session.removeInput(old) }
        for old in session.outputs { session.removeOutput(old) }
        input = nil
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

        guard let format = Self.pickMaxFormat(cam) else {
            throw CaptureError.configurationFailed("no suitable ultra-wide format")
        }

        do {
            try cam.lockForConfiguration()
            cam.activeFormat = format
            if useDualHalfZoom {
                let z = cam.minAvailableVideoZoomFactor
                cam.videoZoomFactor = min(max(z, 0.5), cam.maxAvailableVideoZoomFactor)
            }
            // Prefer 30 fps at max res when the format allows it.
            let duration = CMTime(value: 1, timescale: 30)
            if format.videoSupportedFrameRateRanges.contains(where: {
                $0.minFrameDuration <= duration && duration <= $0.maxFrameDuration
            }) {
                cam.activeVideoMinFrameDuration = duration
                cam.activeVideoMaxFrameDuration = duration
            }
            let dims = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
            dimensions = (Int(dims.width), Int(dims.height))
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
        input = newInput
        device = cam

        videoOut.alwaysDiscardsLateVideoFrames = true
        videoOut.videoSettings = [
            kCVPixelBufferPixelFormatTypeKey as String:
                kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
        ]
        videoOut.setSampleBufferDelegate(self, queue: queue)
        guard session.canAddOutput(videoOut) else {
            throw CaptureError.configurationFailed("cannot add video output")
        }
        session.addOutput(videoOut)

        if let conn = videoOut.connection(with: .video) {
            if #available(iOS 17.0, *) {
                if conn.isVideoRotationAngleSupported(90) {
                    conn.videoRotationAngle = 90
                }
            } else if conn.isVideoOrientationSupported {
                conn.videoOrientation = .portrait
            }
        }

        if previewLayer == nil {
            let preview = AVCaptureVideoPreviewLayer(session: session)
            preview.videoGravity = .resizeAspectFill
            previewLayer = preview
        } else {
            previewLayer?.session = session
        }
    }

    /// Highest-resolution format that still supports ≥30 fps.
    private static func pickMaxFormat(_ device: AVCaptureDevice) -> AVCaptureDevice.Format? {
        let candidates = device.formats.filter { format in
            let dims = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
            let okFps = format.videoSupportedFrameRateRanges.contains {
                $0.maxFrameRate >= 29.5
            }
            return okFps && dims.width >= 1280
        }
        return candidates.max { a, b in
            let da = CMVideoFormatDescriptionGetDimensions(a.formatDescription)
            let db = CMVideoFormatDescriptionGetDimensions(b.formatDescription)
            let aa = Int(da.width) * Int(da.height)
            let bb = Int(db.width) * Int(db.height)
            if aa != bb { return aa < bb }
            // Prefer 4:3-ish room capture when area ties.
            let ra = abs(Float(da.width) / Float(max(1, da.height)) - 4.0 / 3.0)
            let rb = abs(Float(db.width) / Float(max(1, db.height)) - 4.0 / 3.0)
            return ra > rb
        }
    }

    private func applyLockISO() {
        guard let device, device.isExposureModeSupported(.custom) else { return }
        do {
            try device.lockForConfiguration()
            defer { device.unlockForConfiguration() }
            let iso = min(device.activeFormat.maxISO,
                          max(device.activeFormat.minISO, device.iso))
            let duration = device.exposureDuration
            device.setExposureModeCustom(duration: duration, iso: iso,
                                         completionHandler: nil)
            exposureLock.lock()
            axisLock = .iso
            frozenISO = iso
            frozenShutter = nil
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
            axisLock = .shutter
            frozenShutter = duration
            frozenISO = nil
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
        axisLock = .none
        frozenISO = nil
        frozenShutter = nil
        exposureLock.unlock()
    }

    private func steerShutter(device: AVCaptureDevice, iso: Float,
                              stats: FrameAnalysis.LumaStats) {
        let minS = device.activeFormat.minExposureDuration.seconds
        let frameS = device.activeVideoMaxFrameDuration.seconds
        let formatMax = device.activeFormat.maxExposureDuration.seconds
        let maxS = min(formatMax, frameS > 0 ? frameS : formatMax)
        var seconds = device.exposureDuration.seconds
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
        } catch {}
    }

    private func steerISO(device: AVCaptureDevice, shutter: CMTime,
                          stats: FrameAnalysis.LumaStats) {
        let minISO = device.activeFormat.minISO
        let maxISO = device.activeFormat.maxISO
        var iso = device.iso
        if stats.overexposedFraction > 0.03 {
            iso *= 0.72
        } else if stats.meanLuma > 1 {
            let ratio = Float(110.0 / stats.meanLuma)
            iso *= min(1.22, max(0.82, ratio))
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
        // Steer at most ~6.5 Hz so max-res luma does not run every frame.
        exposureLock.lock()
        let mode = axisLock
        let due = mode != .none
            && ProcessInfo.processInfo.systemUptime - lastSteerUptime >= 0.15
        exposureLock.unlock()
        if due {
            steerExposure(stats: FrameAnalysis.lumaStats(of: buf))
        }
        onFrame?(buf, t)
    }
}

import AVFoundation
import CoreMedia
import CoreVideo
import Foundation
import UIKit

/// True 0.5× rear ultra-wide via AVCapture. ARKit cannot drive this lens for
/// world tracking, so preview + saved frames come from here.
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
    private(set) var previewLayer: AVCaptureVideoPreviewLayer?
    private(set) var dimensions: (width: Int, height: Int) = (1920, 1440)

    /// Called on `queue` for every frame.
    var onFrame: ((CVPixelBuffer, CMTime) -> Void)?

    func start() throws {
        let auth = AVCaptureDevice.authorizationStatus(for: .video)
        if auth == .notDetermined {
            // Caller should have requested already; block briefly once.
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
        // Never startRunning inside queue.sync — the first frame callback is
        // delivered on this queue and would deadlock.
        queue.async { [session] in
            if !session.isRunning { session.startRunning() }
        }
    }

    func stop() {
        onFrame = nil
        queue.async { [session] in
            if session.isRunning { session.stopRunning() }
        }
    }

    private func configureLocked() throws {
        if session.isRunning { session.stopRunning() }

        session.beginConfiguration()
        defer { session.commitConfiguration() }

        session.sessionPreset = .inputPriority

        for old in session.inputs { session.removeInput(old) }
        for old in session.outputs { session.removeOutput(old) }
        input = nil

        let device: AVCaptureDevice
        let useDualHalfZoom: Bool
        if let ultra = AVCaptureDevice.default(
            .builtInUltraWideCamera, for: .video, position: .back) {
            device = ultra
            useDualHalfZoom = false
        } else if let dual = AVCaptureDevice.default(
            .builtInDualWideCamera, for: .video, position: .back) {
            device = dual
            useDualHalfZoom = true
        } else {
            throw CaptureError.noUltraWide
        }

        guard let format = Self.pickFormat(device) else {
            throw CaptureError.configurationFailed("no suitable ultra-wide format")
        }

        do {
            try device.lockForConfiguration()
            device.activeFormat = format
            if useDualHalfZoom {
                let z = device.minAvailableVideoZoomFactor
                device.videoZoomFactor = min(max(z, 0.5), device.maxAvailableVideoZoomFactor)
            }
            // Stay at 30 fps — unsupported durations throw ObjC exceptions.
            let duration = CMTime(value: 1, timescale: 30)
            if format.videoSupportedFrameRateRanges.contains(where: {
                $0.minFrameDuration <= duration && duration <= $0.maxFrameDuration
            }) {
                device.activeVideoMinFrameDuration = duration
                device.activeVideoMaxFrameDuration = duration
            }
            let dims = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
            dimensions = (Int(dims.width), Int(dims.height))
            device.unlockForConfiguration()
        } catch {
            throw CaptureError.configurationFailed(
                "device lock: \(error.localizedDescription)")
        }

        let newInput: AVCaptureDeviceInput
        do {
            newInput = try AVCaptureDeviceInput(device: device)
        } catch {
            throw CaptureError.configurationFailed(
                "input: \(error.localizedDescription)")
        }
        guard session.canAddInput(newInput) else {
            throw CaptureError.configurationFailed("cannot add ultra-wide input")
        }
        session.addInput(newInput)
        input = newInput

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

    /// Prefer ~1920×1440 (or closest ≤1920 wide). Max-res 4K copies at 60 fps
    /// were jet-sam crashing the app on Start.
    private static func pickFormat(_ device: AVCaptureDevice) -> AVCaptureDevice.Format? {
        let usable = device.formats.filter { format in
            let dims = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
            let okFps = format.videoSupportedFrameRateRanges.contains {
                $0.maxFrameRate >= 29.5 && $0.minFrameRate <= 30.5
            }
            return okFps && dims.width >= 1280 && dims.width <= 1920
        }
        if let best = usable.max(by: {
            area($0) < area($1)
        }) {
            return best
        }
        // Fall back: smallest format that is still ≥1280 wide at 30 fps.
        return device.formats
            .filter {
                let dims = CMVideoFormatDescriptionGetDimensions($0.formatDescription)
                let okFps = $0.videoSupportedFrameRateRanges.contains {
                    $0.maxFrameRate >= 29.5
                }
                return okFps && dims.width >= 1280
            }
            .min(by: { area($0) < area($1) })
    }

    private static func area(_ format: AVCaptureDevice.Format) -> Int {
        let d = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
        return Int(d.width) * Int(d.height)
    }
}

extension UltraWideCapture: AVCaptureVideoDataOutputSampleBufferDelegate {
    func captureOutput(_ output: AVCaptureOutput,
                       didOutput sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) {
        guard let buf = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        let t = CMSampleBufferGetPresentationTimeStamp(sampleBuffer)
        onFrame?(buf, t)
    }
}

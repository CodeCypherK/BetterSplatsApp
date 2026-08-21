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
        case configurationFailed(String)
        var errorDescription: String? {
            switch self {
            case .noUltraWide:
                return "No ultra-wide (0.5×) camera on this device"
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
    private(set) var previewLayer: AVCaptureVideoPreviewLayer?
    private(set) var dimensions: (width: Int, height: Int) = (1920, 1080)

    /// Called on `queue` for every frame.
    var onFrame: ((CVPixelBuffer, CMTime) -> Void)?

    func start() throws {
        session.beginConfiguration()
        defer { session.commitConfiguration() }
        session.sessionPreset = .inputPriority

        for old in session.inputs { session.removeInput(old) }
        for old in session.outputs { session.removeOutput(old) }

        let device: AVCaptureDevice
        let forceHalfZoom: Bool
        if let ultra = AVCaptureDevice.default(
            .builtInUltraWideCamera, for: .video, position: .back) {
            device = ultra
            forceHalfZoom = false
        } else if let dual = AVCaptureDevice.default(
            .builtInDualWideCamera, for: .video, position: .back) {
            device = dual
            forceHalfZoom = true
        } else {
            throw CaptureError.noUltraWide
        }

        let format = Self.pickFormat(device) ?? device.activeFormat
        do {
            try device.lockForConfiguration()
            device.activeFormat = format
            if forceHalfZoom {
                device.videoZoomFactor = max(device.minAvailableVideoZoomFactor, 0.5)
            }
            let dims = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
            dimensions = (Int(dims.width), Int(dims.height))
            let ranges = format.videoSupportedFrameRateRanges
            let maxFps = ranges.map(\.maxFrameRate).max() ?? 30
            let fps: Double = maxFps >= 59.5 ? 60 : 30
            device.activeVideoMinFrameDuration =
                CMTime(value: 1, timescale: CMTimeScale(fps))
            device.activeVideoMaxFrameDuration =
                CMTime(value: 1, timescale: CMTimeScale(fps))
            if device.isGeometricDistortionCorrectionSupported {
                // Keep UW FOV; GDC can crop toward a narrower look.
                device.isGeometricDistortionCorrectionEnabled = false
            }
            device.unlockForConfiguration()
        } catch {
            throw CaptureError.configurationFailed(
                "device lock: \(error.localizedDescription)")
        }

        let input: AVCaptureDeviceInput
        do {
            input = try AVCaptureDeviceInput(device: device)
        } catch {
            throw CaptureError.configurationFailed(
                "input: \(error.localizedDescription)")
        }
        guard session.canAddInput(input) else {
            throw CaptureError.configurationFailed("cannot add ultra-wide input")
        }
        session.addInput(input)

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
            if conn.isVideoOrientationSupported {
                conn.videoOrientation = .portrait
            }
            if conn.isVideoMirroringSupported {
                conn.isVideoMirrored = false
            }
        }

        let preview = AVCaptureVideoPreviewLayer(session: session)
        preview.videoGravity = .resizeAspectFill
        if let conn = preview.connection, conn.isVideoOrientationSupported {
            conn.videoOrientation = .portrait
        }
        previewLayer = preview

        queue.async { self.session.startRunning() }
    }

    func stop() {
        onFrame = nil
        queue.async { [session] in
            if session.isRunning { session.stopRunning() }
        }
    }

    /// Largest ≥1280-wide format that sustains ≥30 fps.
    private static func pickFormat(_ device: AVCaptureDevice) -> AVCaptureDevice.Format? {
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
            return Int(da.width) * Int(da.height) < Int(db.width) * Int(db.height)
        }
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

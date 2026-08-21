import ARKit
import CoreMedia
import CoreVideo
import Foundation
import simd
import UIKit

/// World tracking + preferred ultra-wide video, no mesh. One session gives
/// both frames and poses without fighting AVCapture for the camera.
final class UltraWideARCapture: NSObject, ARSessionDelegate {
    enum CaptureError: LocalizedError {
        case unsupported
        var errorDescription: String? {
            "ARKit world tracking is not available on this device"
        }
    }

    static var isAvailable: Bool {
        ARWorldTrackingConfiguration.isSupported
    }

    private let session = ARSession()
    private let lock = NSLock()
    private var latestPose: simd_float4x4?
    private var trackingOK = false

    private(set) var dimensions: (width: Int, height: Int) = (1920, 1440)

    /// Called on the ARSession delegate queue for every camera frame.
    var onFrame: ((CVPixelBuffer, CMTime, simd_float4x4?, Bool) -> Void)?

    func start() throws {
        guard ARWorldTrackingConfiguration.isSupported else {
            throw CaptureError.unsupported
        }
        let config = ARWorldTrackingConfiguration()
        config.environmentTexturing = .none
        if let ultra = ARWorldTrackingConfiguration.supportedVideoFormats.first(
            where: { $0.captureDeviceType == .builtInUltraWideCamera }) {
            config.videoFormat = ultra
            dimensions = (Int(ultra.imageResolution.width),
                          Int(ultra.imageResolution.height))
        } else if let best = ARWorldTrackingConfiguration.supportedVideoFormats
            .max(by: {
                $0.imageResolution.width * $0.imageResolution.height
                    < $1.imageResolution.width * $1.imageResolution.height
            }) {
            config.videoFormat = best
            dimensions = (Int(best.imageResolution.width),
                          Int(best.imageResolution.height))
        }
        session.delegate = self
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    func stop() {
        session.pause()
        onFrame = nil
    }

    var arSession: ARSession { session }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let ok: Bool
        if case .normal = frame.camera.trackingState { ok = true } else { ok = false }
        let transform = frame.camera.transform
        lock.lock()
        latestPose = transform
        trackingOK = ok
        lock.unlock()
        let t = CMTime(seconds: frame.timestamp, preferredTimescale: 600)
        onFrame?(frame.capturedImage, t, ok ? transform : nil, ok)
    }
}

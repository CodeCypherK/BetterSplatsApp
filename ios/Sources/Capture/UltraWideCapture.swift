import ARKit
import CoreMedia
import CoreVideo
import Foundation
import simd
import UIKit

/// World tracking on the **1× wide** camera (what ARKit actually supports
/// for rear world tracking). Sharpness / exposure gates run on these
/// frames; if 1× is clean we treat that as good enough for reconstruction.
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
    private(set) var dimensions: (width: Int, height: Int) = (1920, 1440)

    /// Called on the ARSession delegate queue for every camera frame.
    var onFrame: ((CVPixelBuffer, CMTime, simd_float4x4?, Bool) -> Void)?

    func start() throws {
        guard ARWorldTrackingConfiguration.isSupported else {
            throw CaptureError.unsupported
        }
        let config = ARWorldTrackingConfiguration()
        config.environmentTexturing = .none

        // Prefer the highest-res **wide** format. Do not select ultra-wide —
        // rear UW is not available for ARWorldTrackingConfiguration, and
        // falling back by max resolution still lands on 1× wide anyway.
        let wide = ARWorldTrackingConfiguration.supportedVideoFormats.filter {
            $0.captureDeviceType == .builtInWideAngleCamera
        }
        let pick = (wide.isEmpty
            ? ARWorldTrackingConfiguration.supportedVideoFormats
            : wide)
            .max(by: {
                $0.imageResolution.width * $0.imageResolution.height
                    < $1.imageResolution.width * $1.imageResolution.height
            })
        if let pick {
            config.videoFormat = pick
            dimensions = (Int(pick.imageResolution.width),
                          Int(pick.imageResolution.height))
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
        let t = CMTime(seconds: frame.timestamp, preferredTimescale: 600)
        onFrame?(frame.capturedImage, t, transform, ok)
    }
}

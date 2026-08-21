import AVFoundation
import SceneKit
import SwiftUI
import simd
import UIKit

struct CapturedPoseSample: Identifiable, Equatable {
    var id: UInt32
    var position: SIMD3<Float>
    var transform: simd_float4x4
}

/// Live ultra-wide preview (AVCapture) — no pose gizmos in the feed.
struct ScanPreview: UIViewRepresentable {
    var previewLayer: AVCaptureVideoPreviewLayer?

    func makeUIView(context: Context) -> PreviewHost {
        let host = PreviewHost()
        host.backgroundColor = .black
        return host
    }

    func updateUIView(_ host: PreviewHost, context: Context) {
        host.attach(previewLayer)
    }

    final class PreviewHost: UIView {
        private weak var attached: AVCaptureVideoPreviewLayer?

        func attach(_ layer: AVCaptureVideoPreviewLayer?) {
            if attached === layer {
                attached?.frame = bounds
                return
            }
            attached?.removeFromSuperlayer()
            attached = nil
            guard let layer else { return }
            // Layer may still be parented elsewhere after a prior scan.
            layer.removeFromSuperlayer()
            layer.frame = bounds
            self.layer.insertSublayer(layer, at: 0)
            attached = layer
        }

        override func layoutSubviews() {
            super.layoutSubviews()
            attached?.frame = bounds
        }
    }
}

/// Small top-down map of accepted poses and look directions (not in the feed).
struct PoseTrailView: UIViewRepresentable {
    var poses: [CapturedPoseSample]

    func makeUIView(context: Context) -> SCNView {
        let view = SCNView(frame: .zero)
        view.scene = SCNScene()
        view.backgroundColor = UIColor.black.withAlphaComponent(0.45)
        view.allowsCameraControl = false
        view.autoenablesDefaultLighting = false
        view.isPlaying = true
        let cam = SCNNode()
        cam.camera = SCNCamera()
        cam.camera?.usesOrthographicProjection = true
        cam.camera?.orthographicScale = 3.5
        cam.eulerAngles.x = -.pi / 2
        cam.position = SCNVector3(0, 8, 0)
        view.scene?.rootNode.addChildNode(cam)
        view.pointOfView = cam
        context.coordinator.cameraNode = cam
        return view
    }

    func updateUIView(_ view: SCNView, context: Context) {
        context.coordinator.rebuild(poses: poses, in: view)
    }

    func makeCoordinator() -> Coordinator { Coordinator() }

    final class Coordinator {
        var cameraNode: SCNNode?
        private var root = SCNNode()
        private var attached = false

        func rebuild(poses: [CapturedPoseSample], in view: SCNView) {
            guard let scene = view.scene else { return }
            if !attached {
                scene.rootNode.addChildNode(root)
                attached = true
            }
            root.childNodes.forEach { $0.removeFromParentNode() }
            guard !poses.isEmpty else { return }

            for (i, sample) in poses.enumerated() {
                let p = sample.position
                let pin = SCNNode(geometry: SCNSphere(radius: 0.06))
                pin.geometry?.firstMaterial?.diffuse.contents = UIColor.systemYellow
                pin.geometry?.firstMaterial?.lightingModel = .constant
                pin.simdPosition = p
                root.addChildNode(pin)

                let forward = -SIMD3(sample.transform.columns.2.x,
                                     sample.transform.columns.2.y,
                                     sample.transform.columns.2.z)
                let tip = p + simd_normalize(forward) * 0.4
                root.addChildNode(line(from: p, to: tip, color: .cyan))
                if i > 0 {
                    root.addChildNode(line(from: poses[i - 1].position, to: p,
                                           color: UIColor.white.withAlphaComponent(0.45)))
                }
            }

            let xs = poses.map(\.position.x)
            let zs = poses.map(\.position.z)
            let minX = xs.min()!, maxX = xs.max()!
            let minZ = zs.min()!, maxZ = zs.max()!
            let cx = (minX + maxX) / 2
            let cz = (minZ + maxZ) / 2
            let span = max(maxX - minX, maxZ - minZ, 1.5) * 0.7 + 1.2
            cameraNode?.position = SCNVector3(cx, 8, cz)
            cameraNode?.camera?.orthographicScale = CGFloat(span)
        }

        private func line(from a: SIMD3<Float>, to b: SIMD3<Float>,
                          color: UIColor) -> SCNNode {
            var positions: [Float] = [a.x, a.y, a.z, b.x, b.y, b.z]
            let src = SCNGeometrySource(
                data: Data(bytes: &positions, count: positions.count * 4),
                semantic: .vertex, vectorCount: 2, usesFloatComponents: true,
                componentsPerVector: 3, bytesPerComponent: 4,
                dataOffset: 0, dataStride: 12)
            let element = SCNGeometryElement(indices: [Int32(0), Int32(1)],
                                             primitiveType: .line)
            let geo = SCNGeometry(sources: [src], elements: [element])
            geo.firstMaterial?.diffuse.contents = color
            geo.firstMaterial?.lightingModel = .constant
            return SCNNode(geometry: geo)
        }
    }
}

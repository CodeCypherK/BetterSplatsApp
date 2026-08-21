import ARKit
import SceneKit
import SwiftUI

/// Camera preview from the AR session (no mesh) plus a small pose trail.
struct ScanPreview: UIViewRepresentable {
    var session: ARSession?
    var poses: [CapturedPoseSample]

    func makeCoordinator() -> Coordinator { Coordinator() }

    func makeUIView(context: Context) -> ARSCNView {
        let view = ARSCNView(frame: .zero)
        view.scene = SCNScene()
        view.automaticallyUpdatesLighting = false
        view.rendersCameraGrain = false
        view.debugOptions = []
        // Camera feed only — empty scene.
        context.coordinator.trailRoot.name = "trail"
        view.scene.rootNode.addChildNode(context.coordinator.trailRoot)
        return view
    }

    func updateUIView(_ view: ARSCNView, context: Context) {
        if let session, view.session !== session {
            view.session = session
        }
        context.coordinator.rebuild(poses: poses)
    }

    final class Coordinator {
        let trailRoot = SCNNode()

        func rebuild(poses: [CapturedPoseSample]) {
            trailRoot.childNodes.forEach { $0.removeFromParentNode() }
            guard poses.count >= 1 else { return }

            for (i, sample) in poses.enumerated() {
                let pin = SCNNode(geometry: SCNSphere(radius: 0.04))
                pin.geometry?.firstMaterial?.diffuse.contents = UIColor.systemYellow
                pin.geometry?.firstMaterial?.lightingModel = .constant
                pin.simdTransform = sample.transform
                trailRoot.addChildNode(pin)

                let forward = -SIMD3(sample.transform.columns.2.x,
                                     sample.transform.columns.2.y,
                                     sample.transform.columns.2.z)
                let origin = sample.position
                let tip = origin + simd_normalize(forward) * 0.3
                trailRoot.addChildNode(line(from: origin, to: tip, color: .cyan))
                if i > 0 {
                    trailRoot.addChildNode(
                        line(from: poses[i - 1].position, to: origin,
                             color: UIColor.white.withAlphaComponent(0.45)))
                }
            }
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

import ARKit
import SceneKit
import SwiftUI
import UIKit

/// Full-screen AR view: live camera + meshed scan grid + photo pins.
struct CoverageARView: UIViewRepresentable {
    var model: ScanViewModel

    func makeCoordinator() -> Coordinator { Coordinator(model: model) }

    func makeUIView(context: Context) -> ARSCNView {
        let view = ARSCNView(frame: .zero)
        view.delegate = context.coordinator
        view.session.delegate = context.coordinator
        view.automaticallyUpdatesLighting = false
        view.rendersCameraGrain = false
        view.scene = SCNScene()
        view.backgroundColor = .black
        view.debugOptions = []
        context.coordinator.view = view
        context.coordinator.model = model
        DispatchQueue.main.async {
            self.model.arCoordinator = context.coordinator
            context.coordinator.startSessionIfNeeded()
        }
        return view
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {
        context.coordinator.model = model
        context.coordinator.refreshPhotoMarkers()
        context.coordinator.recolorMeshes()
    }


    final class Coordinator: NSObject, ARSCNViewDelegate, ARSessionDelegate {
        var model: ScanViewModel
        weak var view: ARSCNView?
        private var meshNodes: [UUID: SCNNode] = [:]
        private var photoNodes: [UInt32: SCNNode] = [:]
        private var meshAnchors: [UUID: ARMeshAnchor] = [:]
        private var started = false
        private var lastCoverageRefresh: TimeInterval = 0

        init(model: ScanViewModel) { self.model = model }

        func startSessionIfNeeded() {
            guard !started, let view else { return }
            guard ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh)
            else {
                DispatchQueue.main.async {
                    self.model.fail("This device cannot build an ARKit mesh "
                                    + "(needs a LiDAR iPhone).")
                }
                return
            }
            let config = ARWorldTrackingConfiguration()
            config.sceneReconstruction = .mesh
            config.environmentTexturing = .none
            config.frameSemantics = []
            if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
                // Depth stays on-device for meshing quality only — never written.
            }
            view.session.run(config, options: [.resetTracking, .removeExistingAnchors])
            started = true
            DispatchQueue.main.async { self.model.sessionStarted() }
        }

        func pause() {
            view?.session.pause()
            started = false
        }

        // MARK: - Mesh nodes

        func renderer(_ renderer: SCNSceneRenderer, nodeFor anchor: ARAnchor)
            -> SCNNode? {
            guard let mesh = anchor as? ARMeshAnchor else { return nil }
            meshAnchors[mesh.identifier] = mesh
            let thin = model.thinAnchorIds.contains(mesh.identifier)
            let geo = CoverageMeshBuilder.geometry(from: mesh, thin: thin)
            let node = SCNNode(geometry: geo)
            meshNodes[mesh.identifier] = node
            scheduleCoverageRefresh()
            return node
        }

        func renderer(_ renderer: SCNSceneRenderer, didUpdate node: SCNNode,
                      for anchor: ARAnchor) {
            guard let mesh = anchor as? ARMeshAnchor else { return }
            meshAnchors[mesh.identifier] = mesh
            let thin = model.thinAnchorIds.contains(mesh.identifier)
            node.geometry = CoverageMeshBuilder.geometry(from: mesh, thin: thin)
            scheduleCoverageRefresh()
        }

        func renderer(_ renderer: SCNSceneRenderer, didRemove node: SCNNode,
                      for anchor: ARAnchor) {
            meshAnchors.removeValue(forKey: anchor.identifier)
            meshNodes.removeValue(forKey: anchor.identifier)
            scheduleCoverageRefresh()
        }

        func recolorMeshes() {
            for (id, node) in meshNodes {
                guard let mesh = meshAnchors[id] else { continue }
                let thin = model.thinAnchorIds.contains(id)
                node.geometry = CoverageMeshBuilder.geometry(from: mesh, thin: thin)
            }
        }

        func refreshPhotoMarkers() {
            guard let root = view?.scene.rootNode else { return }
            let ids = Set(model.photos.map(\.id))
            for (id, node) in photoNodes where !ids.contains(id) {
                node.removeFromParentNode()
                photoNodes.removeValue(forKey: id)
            }
            for sample in model.photos {
                if let existing = photoNodes[sample.id] {
                    existing.simdTransform = sample.transform
                    continue
                }
                let node = CoverageMeshBuilder.photoMarkerNode()
                node.simdTransform = sample.transform
                root.addChildNode(node)
                photoNodes[sample.id] = node
            }
        }

        private func scheduleCoverageRefresh() {
            let now = CACurrentMediaTime()
            guard now - lastCoverageRefresh > 0.35 else { return }
            lastCoverageRefresh = now
            let anchors = Array(meshAnchors.values)
            let photos = model.photos
            DispatchQueue.global(qos: .userInitiated).async {
                let (stats, thin) = CoverageAnalyzer.analyze(anchors: anchors,
                                                             photos: photos)
                DispatchQueue.main.async {
                    self.model.applyCoverage(stats: stats, thinIds: thin)
                }
            }
        }

        // MARK: - Frame / capture

        func session(_ session: ARSession, didUpdate frame: ARFrame) {
            model.noteFrame(frame)
        }

        func session(_ session: ARSession, didFailWithError error: Error) {
            DispatchQueue.main.async {
                self.model.fail(error.localizedDescription)
            }
        }

        func sessionWasInterrupted(_ session: ARSession) {
            DispatchQueue.main.async { self.model.guidance = "AR interrupted" }
        }

        func sessionInterruptionEnded(_ session: ARSession) {
            startSessionIfNeeded()
        }

        func captureJPEG(encoder: JpegEncoder)
            -> (Data, simd_float4x4, ARCamera)? {
            guard let frame = view?.session.currentFrame else { return nil }
            let buffer = frame.capturedImage
            // Portrait app: ARKit buffer is landscape-right relative to device.
            guard let jpeg = encoder.encodeYUV(buffer, orientation: .right)
            else { return nil }
            return (jpeg, frame.camera.transform, frame.camera)
        }
    }
}

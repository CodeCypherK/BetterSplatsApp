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
        context.coordinator.sync(model: model,
                                 thinIds: model.thinAnchorIds,
                                 photos: model.photos)
        DispatchQueue.main.async {
            self.model.arCoordinator = context.coordinator
            context.coordinator.startSessionIfNeeded()
        }
        return view
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {
        context.coordinator.sync(model: model,
                                 thinIds: model.thinAnchorIds,
                                 photos: model.photos)
        context.coordinator.refreshPhotoMarkers()
        context.coordinator.recolorMeshes()
    }

    final class Coordinator: NSObject, ARSCNViewDelegate, ARSessionDelegate {
        /// Snapshots of MainActor model state for ARKit callback threads.
        private var thinIds = Set<UUID>()
        private var photoSamples: [CapturedPoseSample] = []
        private weak var modelRef: ScanViewModel?

        weak var view: ARSCNView?
        private var meshNodes: [UUID: SCNNode] = [:]
        private var photoNodes: [UInt32: SCNNode] = [:]
        private var meshAnchors: [UUID: ARMeshAnchor] = [:]
        private var started = false
        private var lastCoverageRefresh: TimeInterval = 0

        init(model: ScanViewModel) {
            modelRef = model
            super.init()
        }

        /// Caller must already be on the main actor and pass copied snapshots.
        func sync(model: ScanViewModel, thinIds: Set<UUID>,
                  photos: [CapturedPoseSample]) {
            modelRef = model
            self.thinIds = thinIds
            photoSamples = photos
        }

        func startSessionIfNeeded() {
            guard !started, let view else { return }
            guard ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh)
            else {
                DispatchQueue.main.async {
                    self.modelRef?.fail(
                        "This device cannot build an ARKit mesh "
                        + "(needs a LiDAR iPhone).")
                }
                return
            }
            let config = ARWorldTrackingConfiguration()
            config.sceneReconstruction = .mesh
            config.environmentTexturing = .none
            config.frameSemantics = []
            view.session.run(config, options: [.resetTracking, .removeExistingAnchors])
            started = true
            DispatchQueue.main.async { self.modelRef?.sessionStarted() }
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
            let thin = thinIds.contains(mesh.identifier)
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
            let thin = thinIds.contains(mesh.identifier)
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
                let thin = thinIds.contains(id)
                node.geometry = CoverageMeshBuilder.geometry(from: mesh, thin: thin)
            }
        }

        func refreshPhotoMarkers() {
            guard let root = view?.scene.rootNode else { return }
            let ids = Set(photoSamples.map(\.id))
            for (id, node) in photoNodes where !ids.contains(id) {
                node.removeFromParentNode()
                photoNodes.removeValue(forKey: id)
            }
            for sample in photoSamples {
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
            let photos = photoSamples
            DispatchQueue.global(qos: .userInitiated).async {
                let (stats, thin) = CoverageAnalyzer.analyze(anchors: anchors,
                                                             photos: photos)
                DispatchQueue.main.async {
                    self.modelRef?.applyCoverage(stats: stats, thinIds: thin)
                }
            }
        }

        // MARK: - Frame / capture

        func session(_ session: ARSession, didUpdate frame: ARFrame) {
            let transform = frame.camera.transform
            let ok = frame.camera.trackingState == .normal
            DispatchQueue.main.async {
                self.modelRef?.noteFrame(transform: transform, trackingOK: ok)
            }
        }

        func session(_ session: ARSession, didFailWithError error: Error) {
            DispatchQueue.main.async {
                self.modelRef?.fail(error.localizedDescription)
            }
        }

        func sessionWasInterrupted(_ session: ARSession) {
            DispatchQueue.main.async {
                self.modelRef?.guidance = "AR interrupted"
            }
        }

        func sessionInterruptionEnded(_ session: ARSession) {
            startSessionIfNeeded()
        }

        func captureJPEG(encoder: JpegEncoder)
            -> (Data, simd_float4x4, ARCamera)? {
            guard let frame = view?.session.currentFrame else { return nil }
            let buffer = frame.capturedImage
            guard let jpeg = encoder.encodeYUV(buffer, orientation: .right)
            else { return nil }
            return (jpeg, frame.camera.transform, frame.camera)
        }
    }
}

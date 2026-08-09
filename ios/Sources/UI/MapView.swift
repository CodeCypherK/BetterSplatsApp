import MetalKit
import SwiftUI

/// Live 3D diagnostic view: sparse points, keyframe frusta, weak-area
/// markers. Orbit with drag, zoom with pinch.
struct MapView: View {
    let model: CaptureViewModel

    var body: some View {
        MapSurface(renderer: model.mapRenderer)
            .ignoresSafeArea(edges: .bottom)
            .navigationTitle("Live Map")
            .navigationBarTitleDisplayMode(.inline)
            .onAppear { model.refreshSnapshot() }
    }
}

private struct MapSurface: UIViewRepresentable {
    let renderer: MapRenderer

    func makeUIView(context: Context) -> MTKView {
        let view = MTKView()
        view.preferredFramesPerSecond = 30
        view.clearColor = MTLClearColor(red: 0.04, green: 0.05, blue: 0.08, alpha: 1)
        renderer.attach(to: view)
        view.delegate = renderer

        let pan = UIPanGestureRecognizer(
            target: context.coordinator, action: #selector(Coordinator.pan(_:)))
        view.addGestureRecognizer(pan)
        let pinch = UIPinchGestureRecognizer(
            target: context.coordinator, action: #selector(Coordinator.pinch(_:)))
        view.addGestureRecognizer(pinch)
        return view
    }

    func updateUIView(_ uiView: MTKView, context: Context) {}

    func makeCoordinator() -> Coordinator { Coordinator(renderer: renderer) }

    final class Coordinator: NSObject {
        let renderer: MapRenderer
        private var lastYaw: Float = 0
        private var lastPitch: Float = 0
        private var lastScale: Float = 1

        init(renderer: MapRenderer) { self.renderer = renderer }

        @objc func pan(_ gesture: UIPanGestureRecognizer) {
            if gesture.state == .began {
                lastYaw = renderer.yaw
                lastPitch = renderer.pitch
            }
            let translation = gesture.translation(in: gesture.view)
            renderer.yaw = lastYaw + Float(translation.x) * 0.008
            renderer.pitch = max(-1.4, min(1.4, lastPitch + Float(translation.y) * 0.008))
        }

        @objc func pinch(_ gesture: UIPinchGestureRecognizer) {
            if gesture.state == .began { lastScale = renderer.distanceScale }
            renderer.distanceScale =
                max(0.3, min(5, lastScale / Float(gesture.scale)))
        }
    }
}

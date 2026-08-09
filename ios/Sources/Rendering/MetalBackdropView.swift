import MetalKit
import SwiftUI

/// Minimal MTKView wrapper: clears to a dark gradient-ish tone every frame.
/// Exists in M0 purely to prove a Metal device + drawable pipeline runs on
/// the target device; the capture preview and point-cloud renderers build on
/// this in M1/M5.
struct MetalBackdropView: UIViewRepresentable {
    func makeCoordinator() -> Renderer { Renderer() }

    func makeUIView(context: Context) -> MTKView {
        let view = MTKView()
        view.device = MTLCreateSystemDefaultDevice()
        view.delegate = context.coordinator
        view.preferredFramesPerSecond = 30
        view.clearColor = MTLClearColor(red: 0.05, green: 0.06, blue: 0.09, alpha: 1)
        view.isPaused = view.device == nil
        return view
    }

    func updateUIView(_ uiView: MTKView, context: Context) {}

    final class Renderer: NSObject, MTKViewDelegate {
        private var commandQueue: MTLCommandQueue?

        func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

        func draw(in view: MTKView) {
            guard let device = view.device,
                  let drawable = view.currentDrawable,
                  let descriptor = view.currentRenderPassDescriptor else { return }
            if commandQueue == nil { commandQueue = device.makeCommandQueue() }
            guard let queue = commandQueue,
                  let commands = queue.makeCommandBuffer(),
                  let encoder = commands.makeRenderCommandEncoder(descriptor: descriptor)
            else { return }
            encoder.endEncoding()
            commands.present(drawable)
            commands.commit()
        }
    }
}

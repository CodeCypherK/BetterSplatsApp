import CoreVideo
import Metal
import MetalKit
import simd

/// Renders capture pixel buffers (420f bi-planar YUV, sensor-landscape) into
/// an MTKView rotated for portrait display. This is the base of the capture
/// screen's render stack; pose-projected overlays draw into the same pass
/// from M5 on.
///
/// The buffers are handed to the preview EXACTLY as the camera produced
/// them, and the rotation lives only in the texture coordinates. That is
/// deliberate and load-bearing: rotating the buffers themselves — or asking
/// AVFoundation to deliver them rotated via `videoRotationAngle` — would
/// change their dimensions, and the per-frame intrinsics the engine and the
/// stored `meta.json` carry are expressed against the delivered buffer. A
/// rotated buffer with unrotated intrinsics is a camera that does not exist,
/// and it would fail silently as drift rather than loudly as a wrong-looking
/// picture. Display is display; the reconstruction sees the sensor.
final class VideoPreviewRenderer: NSObject, MTKViewDelegate {
    private var device: MTLDevice?
    private var queue: MTLCommandQueue?
    private var pipeline: MTLRenderPipelineState?
    private var textureCache: CVMetalTextureCache?

    private let bufferLock = NSLock()
    private var pendingBuffer: CVPixelBuffer?
    /// Recomputed only when the source or drawable size changes, which is
    /// almost never — but it is four float2s, so cheapness is not the point;
    /// keeping the derivation in one place is.
    private var texcoords: [SIMD2<Float>] = []
    private var texcoordKey: (Int, Int, Int, Int) = (0, 0, 0, 0)

    /// Called from the capture queue with the newest frame; keeps only the
    /// latest (display drops are fine, storage never routes through here).
    func enqueue(_ pixelBuffer: CVPixelBuffer) {
        bufferLock.lock()
        pendingBuffer = pixelBuffer
        bufferLock.unlock()
    }

    func attach(to view: MTKView) {
        let device = view.device ?? MTLCreateSystemDefaultDevice()
        guard let device else { return }
        view.device = device
        self.device = device
        queue = device.makeCommandQueue()
        CVMetalTextureCacheCreate(nil, nil, device, nil, &textureCache)

        guard let library = device.makeDefaultLibrary(),
              let vertexFn = library.makeFunction(name: "preview_vertex"),
              let fragmentFn = library.makeFunction(name: "preview_fragment")
        else { return }
        let desc = MTLRenderPipelineDescriptor()
        desc.vertexFunction = vertexFn
        desc.fragmentFunction = fragmentFn
        desc.colorAttachments[0].pixelFormat = view.colorPixelFormat
        pipeline = try? device.makeRenderPipelineState(descriptor: desc)
    }

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

    func draw(in view: MTKView) {
        guard let queue,
              let drawable = view.currentDrawable,
              let descriptor = view.currentRenderPassDescriptor,
              let commands = queue.makeCommandBuffer()
        else { return }

        bufferLock.lock()
        let buffer = pendingBuffer
        bufferLock.unlock()

        guard let buffer, let pipeline, let cache = textureCache,
              let encoder = commands.makeRenderCommandEncoder(descriptor: descriptor)
        else {
            // No frame yet: clear only.
            if let encoder = commands.makeRenderCommandEncoder(descriptor: descriptor) {
                encoder.endEncoding()
            }
            commands.present(drawable)
            commands.commit()
            return
        }

        var yTextureRef: CVMetalTexture?
        var cbcrTextureRef: CVMetalTexture?
        let yWidth = CVPixelBufferGetWidthOfPlane(buffer, 0)
        let yHeight = CVPixelBufferGetHeightOfPlane(buffer, 0)
        CVMetalTextureCacheCreateTextureFromImage(
            nil, cache, buffer, nil, .r8Unorm, yWidth, yHeight, 0, &yTextureRef)
        CVMetalTextureCacheCreateTextureFromImage(
            nil, cache, buffer, nil, .rg8Unorm,
            CVPixelBufferGetWidthOfPlane(buffer, 1),
            CVPixelBufferGetHeightOfPlane(buffer, 1), 1, &cbcrTextureRef)

        if let yRef = yTextureRef, let cbcrRef = cbcrTextureRef,
           let yTexture = CVMetalTextureGetTexture(yRef),
           let cbcrTexture = CVMetalTextureGetTexture(cbcrRef) {
            updateTexcoords(sourceWidth: yWidth, sourceHeight: yHeight,
                            drawableSize: view.drawableSize)
            encoder.setRenderPipelineState(pipeline)
            texcoords.withUnsafeBytes { raw in
                encoder.setVertexBytes(raw.baseAddress!, length: raw.count,
                                       index: 0)
            }
            encoder.setFragmentTexture(yTexture, index: 0)
            encoder.setFragmentTexture(cbcrTexture, index: 1)
            encoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 4)
        }
        encoder.endEncoding()
        commands.present(drawable)
        commands.commit()
    }

    private func updateTexcoords(sourceWidth: Int, sourceHeight: Int,
                                 drawableSize: CGSize) {
        let key = (sourceWidth, sourceHeight,
                   Int(drawableSize.width.rounded()),
                   Int(drawableSize.height.rounded()))
        if key == texcoordKey && !texcoords.isEmpty { return }
        texcoordKey = key
        texcoords = PreviewOrientation.texcoords(
            sourceWidth: sourceWidth, sourceHeight: sourceHeight,
            drawableSize: drawableSize)
    }
}

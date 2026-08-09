import MetalKit
import simd

/// Renders the live reconstruction snapshot: point cloud, keyframe frusta,
/// and weak-area markers, with an orbit camera. Buffers are refreshed from
/// the engine snapshot at a few Hz by the owning view.
final class MapRenderer: NSObject, MTKViewDelegate {
    private var device: MTLDevice?
    private var queue: MTLCommandQueue?
    private var pointPipeline: MTLRenderPipelineState?
    private var linePipeline: MTLRenderPipelineState?
    private var depthState: MTLDepthStencilState?

    private var pointBuffer: MTLBuffer?
    private var pointColorBuffer: MTLBuffer?
    private var pointCount = 0
    private var lineBuffer: MTLBuffer?
    private var lineColorBuffer: MTLBuffer?
    private var lineCount = 0

    private let stateLock = NSLock()
    private var sceneCenter = SIMD3<Float>(0, 0, 0)
    private var sceneRadius: Float = 3

    // Orbit camera state (updated from gestures on the main thread).
    var yaw: Float = 0.6
    var pitch: Float = -0.4
    var distanceScale: Float = 1.4

    func attach(to view: MTKView) {
        let device = view.device ?? MTLCreateSystemDefaultDevice()
        guard let device else { return }
        view.device = device
        view.depthStencilPixelFormat = .depth32Float
        self.device = device
        queue = device.makeCommandQueue()

        guard let library = device.makeDefaultLibrary(),
              let pointV = library.makeFunction(name: "map_point_vertex"),
              let pointF = library.makeFunction(name: "map_point_fragment"),
              let lineV = library.makeFunction(name: "map_line_vertex"),
              let lineF = library.makeFunction(name: "map_line_fragment")
        else { return }

        let pointDesc = MTLRenderPipelineDescriptor()
        pointDesc.vertexFunction = pointV
        pointDesc.fragmentFunction = pointF
        pointDesc.colorAttachments[0].pixelFormat = view.colorPixelFormat
        pointDesc.depthAttachmentPixelFormat = .depth32Float
        pointPipeline = try? device.makeRenderPipelineState(descriptor: pointDesc)

        let lineDesc = MTLRenderPipelineDescriptor()
        lineDesc.vertexFunction = lineV
        lineDesc.fragmentFunction = lineF
        lineDesc.colorAttachments[0].pixelFormat = view.colorPixelFormat
        lineDesc.depthAttachmentPixelFormat = .depth32Float
        linePipeline = try? device.makeRenderPipelineState(descriptor: lineDesc)

        let depthDesc = MTLDepthStencilDescriptor()
        depthDesc.depthCompareFunction = .less
        depthDesc.isDepthWriteEnabled = true
        depthState = device.makeDepthStencilState(descriptor: depthDesc)
    }

    /// Rebuilds GPU buffers from a snapshot (call on the main thread).
    func update(with snapshot: CoreEngine.Snapshot) {
        guard let device else { return }
        stateLock.lock()
        defer { stateLock.unlock() }

        pointCount = snapshot.points.count
        if pointCount > 0 {
            pointBuffer = device.makeBuffer(
                bytes: snapshot.points,
                length: MemoryLayout<SIMD4<Float>>.stride * pointCount)
            pointColorBuffer = device.makeBuffer(
                bytes: snapshot.pointColors,
                length: MemoryLayout<SIMD4<Float>>.stride * pointCount)
        }

        // Frusta as line lists; weak areas as expanding diamond markers.
        var lines: [SIMD4<Float>] = []
        var colors: [SIMD4<Float>] = []
        let frustumColor = SIMD4<Float>(0.4, 0.75, 1.0, 1.0)
        for camera in snapshot.cameras {
            appendFrustum(camera: camera, into: &lines, colors: &colors,
                          color: frustumColor)
        }
        let weakColor = SIMD4<Float>(1.0, 0.35, 0.25, 1.0)
        for area in snapshot.weakAreas {
            appendDiamond(center: area.center, radius: max(0.12, area.radiusM),
                          into: &lines, colors: &colors, color: weakColor)
        }
        lineCount = lines.count
        if lineCount > 0 {
            lineBuffer = device.makeBuffer(
                bytes: lines, length: MemoryLayout<SIMD4<Float>>.stride * lineCount)
            lineColorBuffer = device.makeBuffer(
                bytes: colors, length: MemoryLayout<SIMD4<Float>>.stride * lineCount)
        }

        let lo = snapshot.boundsMin
        let hi = snapshot.boundsMax
        sceneCenter = (lo + hi) * 0.5
        sceneRadius = max(1.0, simd_length(hi - lo) * 0.5)
    }

    private func appendFrustum(camera: (q: SIMD4<Float>, t: SIMD3<Float>),
                               into lines: inout [SIMD4<Float>],
                               colors: inout [SIMD4<Float>],
                               color: SIMD4<Float>) {
        // Camera center and axes from world-to-camera (q wxyz, t).
        let q = simd_quatf(ix: camera.q.y, iy: camera.q.z, iz: camera.q.w,
                           r: camera.q.x)
        let qInv = q.inverse
        let center = qInv.act(-camera.t)
        let size: Float = 0.08
        let corners = [
            SIMD3<Float>(-size, -size * 0.75, size * 1.6),
            SIMD3<Float>(size, -size * 0.75, size * 1.6),
            SIMD3<Float>(size, size * 0.75, size * 1.6),
            SIMD3<Float>(-size, size * 0.75, size * 1.6),
        ].map { center + qInv.act($0) }

        for corner in corners {
            lines.append(SIMD4(center, 1))
            lines.append(SIMD4(corner, 1))
        }
        for i in 0..<4 {
            lines.append(SIMD4(corners[i], 1))
            lines.append(SIMD4(corners[(i + 1) % 4], 1))
        }
        for _ in 0..<16 { colors.append(color) }
    }

    private func appendDiamond(center: SIMD3<Float>, radius: Float,
                               into lines: inout [SIMD4<Float>],
                               colors: inout [SIMD4<Float>],
                               color: SIMD4<Float>) {
        let axes = [SIMD3<Float>(radius, 0, 0), SIMD3<Float>(0, radius, 0),
                    SIMD3<Float>(0, 0, radius)]
        for i in 0..<3 {
            for j in 0..<3 where i != j {
                for si: Float in [-1, 1] {
                    for sj: Float in [-1, 1] {
                        lines.append(SIMD4(center + axes[i] * si, 1))
                        lines.append(SIMD4(center + axes[j] * sj, 1))
                        colors.append(color)
                        colors.append(color)
                    }
                }
            }
        }
    }

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

    func draw(in view: MTKView) {
        guard let queue,
              let drawable = view.currentDrawable,
              let descriptor = view.currentRenderPassDescriptor,
              let commands = queue.makeCommandBuffer(),
              let encoder = commands.makeRenderCommandEncoder(descriptor: descriptor)
        else { return }

        stateLock.lock()
        var mvp = viewProjection(aspect: Float(view.drawableSize.width /
                                               max(1, view.drawableSize.height)))
        let points = pointBuffer
        let pointColors = pointColorBuffer
        let nPoints = pointCount
        let lines = lineBuffer
        let lineColors = lineColorBuffer
        let nLines = lineCount
        stateLock.unlock()

        if let depthState { encoder.setDepthStencilState(depthState) }
        if let pointPipeline, let points, let pointColors, nPoints > 0 {
            encoder.setRenderPipelineState(pointPipeline)
            encoder.setVertexBuffer(points, offset: 0, index: 0)
            encoder.setVertexBuffer(pointColors, offset: 0, index: 1)
            encoder.setVertexBytes(&mvp, length: MemoryLayout<simd_float4x4>.stride,
                                   index: 2)
            encoder.drawPrimitives(type: .point, vertexStart: 0,
                                   vertexCount: nPoints)
        }
        if let linePipeline, let lines, let lineColors, nLines > 0 {
            encoder.setRenderPipelineState(linePipeline)
            encoder.setVertexBuffer(lines, offset: 0, index: 0)
            encoder.setVertexBuffer(lineColors, offset: 0, index: 1)
            encoder.setVertexBytes(&mvp, length: MemoryLayout<simd_float4x4>.stride,
                                   index: 2)
            encoder.drawPrimitives(type: .line, vertexStart: 0, vertexCount: nLines)
        }
        encoder.endEncoding()
        commands.present(drawable)
        commands.commit()
    }

    private func viewProjection(aspect: Float) -> simd_float4x4 {
        let distance = sceneRadius * 2.2 * distanceScale
        let eye = sceneCenter + SIMD3<Float>(
            distance * cos(pitch) * sin(yaw),
            -distance * sin(pitch),
            distance * cos(pitch) * cos(yaw))
        let viewM = lookAt(eye: eye, center: sceneCenter, up: SIMD3(0, -1, 0))
        let projM = perspective(fovY: 0.9, aspect: aspect, near: 0.05,
                                far: distance * 10)
        return projM * viewM
    }

    private func lookAt(eye: SIMD3<Float>, center: SIMD3<Float>,
                        up: SIMD3<Float>) -> simd_float4x4 {
        let f = simd_normalize(center - eye)
        let s = simd_normalize(simd_cross(f, up))
        let u = simd_cross(s, f)
        return simd_float4x4(columns: (
            SIMD4(s.x, u.x, -f.x, 0),
            SIMD4(s.y, u.y, -f.y, 0),
            SIMD4(s.z, u.z, -f.z, 0),
            SIMD4(-simd_dot(s, eye), -simd_dot(u, eye), simd_dot(f, eye), 1)))
    }

    private func perspective(fovY: Float, aspect: Float, near: Float,
                             far: Float) -> simd_float4x4 {
        let y = 1 / tan(fovY * 0.5)
        let x = y / aspect
        let z = far / (near - far)
        return simd_float4x4(columns: (
            SIMD4(x, 0, 0, 0),
            SIMD4(0, y, 0, 0),
            SIMD4(0, 0, z, -1),
            SIMD4(0, 0, z * near, 0)))
    }
}

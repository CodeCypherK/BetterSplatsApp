import Accelerate
import CoreVideo
import Foundation

/// Hands frames to the live tracker without the capture queue waiting for it.
///
/// The tracker is not a per-frame budget item. One `bs_live_feed` runs ORB
/// extraction, guided matching, PnP RANSAC and a Ceres pose refine, and
/// nothing about that is bounded by the 33 ms between frames — a textureless
/// wall, a relocalization sweep, or a thermally throttled phone all push it
/// past. Called inline on the capture queue (which is what used to happen),
/// every millisecond over budget is charged to capture itself:
///
///   * `AVCaptureDataOutputSynchronizer` discards frames while its delegate
///     is busy, so the app never even sees them — they are missing from the
///     storage gate and from the quality statistics, and nothing counts them.
///   * The preview enqueue sat *after* the feed, so slow tracking showed up
///     to the user as a stuttering viewfinder with no explanation.
///
/// So the feed moves here: the capture queue writes one frame into a
/// single-slot mailbox and returns. If the tracker is still busy when the
/// next frame arrives, the pending one is replaced rather than queued.
///
/// Dropping is the right behaviour and not a compromise. A live pose is only
/// useful while it still describes where the phone is, so a backlog of stale
/// frames is worth less than the newest one — and an unbounded queue on a
/// 30 fps source with a slower consumer ends as a memory crash. What changes
/// is that the drop is now OURS: bounded, counted, and reportable, instead of
/// happening inside AVFoundation where neither the app nor the engine can see
/// it.
final class EngineFeeder: @unchecked Sendable {
    /// One frame, owned outright. The capture callback's pixel buffers are
    /// only valid for the duration of the callback, so everything the engine
    /// needs is copied out before it returns.
    struct Frame {
        var frameId: UInt32
        var tCapture: Double
        var tDepth: Double
        var luma: [UInt8]          // tracking-resolution, tightly packed
        var lumaWidth: Int
        var lumaHeight: Int
        var fx: Double, fy: Double, cx: Double, cy: Double
        var depth: [Float]
        var depthWidth: Int
        var depthHeight: Int
        var dfx: Double, dfy: Double, dcx: Double, dcy: Double
        var gyro: SIMD3<Float>?
    }

    /// Width the engine tracks at. It downsamples anything wider than 1280 to
    /// this itself, so scaling here costs nothing extra overall and shrinks
    /// the copy that crosses the queue by 4x (2.7 MB to 0.7 MB per frame at
    /// 1920x1440). Intrinsics are scaled with it.
    static let trackingWidth = 960

    private let queue = DispatchQueue(label: "bs.engine.feed", qos: .userInitiated)
    private let lock = NSLock()
    private var pending: Frame?
    private var running = false

    private var deliveredCount: UInt32 = 0
    private var fedCount: UInt32 = 0
    private var droppedCount: UInt32 = 0

    /// What the tracker is doing with the frames it is being given.
    struct Counts {
        var delivered: UInt32   // frames the capture queue offered
        var fed: UInt32         // frames the engine actually saw
        var dropped: UInt32     // replaced in the mailbox before being fed
        /// Fraction of offered frames the tracker never saw.
        var dropFraction: Double {
            delivered > 0 ? Double(dropped) / Double(delivered) : 0
        }
    }

    var counts: Counts {
        lock.lock()
        defer { lock.unlock() }
        return Counts(delivered: deliveredCount, fed: fedCount,
                      dropped: droppedCount)
    }

    /// Returns once the tracker has finished every frame it accepted.
    ///
    /// The caller must have stopped the camera first, or new frames keep
    /// arriving and this never settles. Ending a live session while a feed is
    /// in flight is safe — the engine serializes them and rejects a feed
    /// after `bs_live_end` — but it silently discards the last frames of the
    /// capture, which are the ones covering wherever the user finished.
    func finish() async {
        await withCheckedContinuation { continuation in
            queue.async { continuation.resume() }
        }
    }

    /// Offers a frame. Returns immediately; the engine call happens on the
    /// feed queue. Replaces any frame still waiting.
    func offer(_ frame: Frame) {
        lock.lock()
        deliveredCount += 1
        if pending != nil { droppedCount += 1 }
        pending = frame
        let idle = !running
        if idle { running = true }
        lock.unlock()
        guard idle else { return }
        queue.async { [weak self] in self?.drain() }
    }

    private func drain() {
        while true {
            lock.lock()
            guard let frame = pending else {
                running = false
                lock.unlock()
                return
            }
            pending = nil
            fedCount += 1
            lock.unlock()
            feed(frame)
        }
    }

    private func feed(_ frame: Frame) {
        var input = bs_frame_in()
        input.frame_id = frame.frameId
        input.t_capture = frame.tCapture
        input.t_depth = frame.tDepth
        input.luma_width = Int32(frame.lumaWidth)
        input.luma_height = Int32(frame.lumaHeight)
        input.luma_stride = Int32(frame.lumaWidth)
        input.fx = frame.fx
        input.fy = frame.fy
        input.cx = frame.cx
        input.cy = frame.cy
        input.dfx = frame.dfx
        input.dfy = frame.dfy
        input.dcx = frame.dcx
        input.dcy = frame.dcy
        input.depth_width = Int32(frame.depthWidth)
        input.depth_height = Int32(frame.depthHeight)
        if let gyro = frame.gyro {
            input.gyro_dx = gyro.x
            input.gyro_dy = gyro.y
            input.gyro_dz = gyro.z
            input.gyro_valid = 1
        }
        frame.luma.withUnsafeBufferPointer { lumaBuf in
            frame.depth.withUnsafeBufferPointer { depthBuf in
                input.luma = lumaBuf.baseAddress
                input.depth = depthBuf.baseAddress
                _ = CoreEngine.shared.liveFeed(&input)
            }
        }
    }

    /// Copies plane 0 of a bi-planar 420 buffer down to tracking resolution.
    /// The caller must hold the pixel buffer's base address lock.
    ///
    /// Returns the pixels and the scale applied, so the caller can scale the
    /// intrinsics by the same factor — a downscaled image with full-resolution
    /// intrinsics is a camera that does not exist, and it fails silently as
    /// drift rather than loudly as an error.
    static func downscaledLuma(from pixelBuffer: CVPixelBuffer)
        -> (pixels: [UInt8], width: Int, height: Int, scale: Double)? {
        guard let base = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0)
        else { return nil }
        let width = CVPixelBufferGetWidthOfPlane(pixelBuffer, 0)
        let height = CVPixelBufferGetHeightOfPlane(pixelBuffer, 0)
        let stride = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
        guard width > 0, height > 0 else { return nil }

        if width <= trackingWidth {
            var pixels = [UInt8](repeating: 0, count: width * height)
            pixels.withUnsafeMutableBytes { dst in
                for row in 0..<height {
                    dst.baseAddress!.advanced(by: row * width)
                        .copyMemory(from: base.advanced(by: row * stride),
                                    byteCount: width)
                }
            }
            return (pixels, width, height, 1.0)
        }

        let scale = Double(trackingWidth) / Double(width)
        let outWidth = trackingWidth
        let outHeight = max(1, Int((Double(height) * scale).rounded()))
        var pixels = [UInt8](repeating: 0, count: outWidth * outHeight)
        var src = vImage_Buffer(data: base, height: vImagePixelCount(height),
                                width: vImagePixelCount(width), rowBytes: stride)
        let ok = pixels.withUnsafeMutableBytes { dst -> Bool in
            var out = vImage_Buffer(data: dst.baseAddress,
                                    height: vImagePixelCount(outHeight),
                                    width: vImagePixelCount(outWidth),
                                    rowBytes: outWidth)
            return vImageScale_Planar8(&src, &out, nil,
                                       vImage_Flags(kvImageHighQualityResampling))
                == kvImageNoError
        }
        guard ok else { return nil }
        return (pixels, outWidth, outHeight, scale)
    }
}

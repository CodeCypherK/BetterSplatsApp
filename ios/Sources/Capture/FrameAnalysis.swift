import Accelerate
import CoreVideo

/// Cheap per-frame image statistics computed on the capture queue.
/// The keyframe-quality gates in the engine recompute their own metrics;
/// these feed meta.json and the app-side storage gate.
enum FrameAnalysis {
    struct LumaStats {
        var laplacianVariance: Double
        var overexposedFraction: Double
    }

    /// Computes stats on a subsampled grid of the luma plane (plane 0 of a
    /// bi-planar 420 buffer). ~50k samples at 1920x1440 — well under 2 ms.
    static func lumaStats(of pixelBuffer: CVPixelBuffer) -> LumaStats {
        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

        guard let base = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0) else {
            return LumaStats(laplacianVariance: 0, overexposedFraction: 0)
        }
        let width = CVPixelBufferGetWidthOfPlane(pixelBuffer, 0)
        let height = CVPixelBufferGetHeightOfPlane(pixelBuffer, 0)
        let stride = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
        let luma = base.assumingMemoryBound(to: UInt8.self)

        let step = 4  // sample every 4th pixel in both axes
        var lapSum = 0.0
        var lapSqSum = 0.0
        var count = 0
        var overexposed = 0
        var total = 0

        var y = step
        while y < height - step {
            var x = step
            let row = y * stride
            while x < width - step {
                let c = Double(luma[row + x])
                let up = Double(luma[(y - step) * stride + x])
                let down = Double(luma[(y + step) * stride + x])
                let left = Double(luma[row + x - step])
                let right = Double(luma[row + x + step])
                let lap = up + down + left + right - 4.0 * c
                lapSum += lap
                lapSqSum += lap * lap
                count += 1
                if c >= 250 { overexposed += 1 }
                total += 1
                x += step
            }
            y += step
        }

        guard count > 0 else {
            return LumaStats(laplacianVariance: 0, overexposedFraction: 0)
        }
        let mean = lapSum / Double(count)
        let variance = lapSqSum / Double(count) - mean * mean
        return LumaStats(
            laplacianVariance: max(0, variance),
            overexposedFraction: Double(overexposed) / Double(total)
        )
    }
}

/// Extracts raw float16 depth samples and per-frame intrinsics from an
/// AVDepthData (already converted to DepthFloat16 by the capture manager).
enum DepthPacker {
    struct Packed {
        var f16: [UInt16]
        var f32: [Float]
        var width: Int
        var height: Int
    }

    static func pack(_ depthMap: CVPixelBuffer) -> Packed? {
        guard CVPixelBufferGetPixelFormatType(depthMap) == kCVPixelFormatType_DepthFloat16
        else { return nil }
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        guard let base = CVPixelBufferGetBaseAddress(depthMap) else { return nil }
        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        let stride = CVPixelBufferGetBytesPerRow(depthMap)

        var f16 = [UInt16](repeating: 0, count: width * height)
        f16.withUnsafeMutableBytes { dst in
            for row in 0..<height {
                let src = base.advanced(by: row * stride)
                dst.baseAddress!.advanced(by: row * width * 2)
                    .copyMemory(from: src, byteCount: width * 2)
            }
        }

        // Widen to float32 for the engine feed using vImage.
        var f32 = [Float](repeating: 0, count: width * height)
        f16.withUnsafeMutableBufferPointer { halfBuf in
            f32.withUnsafeMutableBufferPointer { floatBuf in
                var src = vImage_Buffer(
                    data: halfBuf.baseAddress!, height: vImagePixelCount(height),
                    width: vImagePixelCount(width), rowBytes: width * 2)
                var dst = vImage_Buffer(
                    data: floatBuf.baseAddress!, height: vImagePixelCount(height),
                    width: vImagePixelCount(width), rowBytes: width * 4)
                vImageConvert_Planar16FtoPlanarF(&src, &dst, 0)
            }
        }
        return Packed(f16: f16, f32: f32, width: width, height: height)
    }
}

import CoreMedia
import CoreVideo
import Foundation
import simd

/// Evaluates every preview frame; accepts at most one JPEG per second, and
/// only when that second's best frame is sharp, well-exposed, and a new
/// viewpoint. Blurry / bad / near-duplicate windows save nothing — rate can
/// fall well below 1 fps.
final class BestFrameGate {
    struct Candidate {
        var pixelBuffer: CVPixelBuffer
        var time: CMTime
        var stats: FrameAnalysis.LumaStats
        var score: Double
        var pose: simd_float4x4?
    }

    struct Accepted {
        var pixelBuffer: CVPixelBuffer
        var time: CMTime
        var stats: FrameAnalysis.LumaStats
        var pose: simd_float4x4?
    }

    /// Hard ceiling: never accept twice within this interval.
    var minIntervalSeconds: Double = 1.0
    /// Must move or turn enough vs the last kept photo.
    var minMoveM: Float = 0.15
    var minTurnRad: Float = 0.18  // ~10°
    var minSharpness: Double = 70
    var maxOverexposed: Double = 0.12
    var lumaRange: ClosedRange<Double> = 28...210

    private var windowStart: Double?
    private var best: Candidate?
    private var lastAcceptedPose: simd_float4x4?
    private var lastAcceptedStats: FrameAnalysis.LumaStats?
    private var lastAcceptTime: Double?

    func consider(pixelBuffer: CVPixelBuffer, time: CMTime,
                  pose: simd_float4x4?) -> Accepted? {
        let t = CMTimeGetSeconds(time)
        if windowStart == nil { windowStart = t }

        // Rank only frames that already pass quality — blurry ones never win.
        // Copy the pixel buffer only when it becomes the new window best so we
        // do not allocate a full-res clone on every good preview frame.
        let stats = FrameAnalysis.lumaStats(of: pixelBuffer)
        if let scored = score(stats: stats) {
            let beats = best == nil
                || scored > best!.score
                || (pose != nil && best?.pose == nil && scored >= best!.score * 0.9)
            if beats, let retained = copyBuffer(pixelBuffer) {
                best = Candidate(pixelBuffer: retained, time: time,
                                 stats: stats, score: scored, pose: pose)
            }
        }

        guard let start = windowStart,
              t - start >= minIntervalSeconds else { return nil }

        // Close the window either way — no obligation to accept.
        let winner = best
        windowStart = t
        best = nil

        guard let winner else { return nil }

        // Absolute 1 fps ceiling (even across window edge jitter).
        if let lastT = lastAcceptTime, t - lastT < minIntervalSeconds {
            return nil
        }
        guard passesNovelty(winner) else { return nil }

        lastAcceptedPose = winner.pose
        lastAcceptedStats = winner.stats
        lastAcceptTime = CMTimeGetSeconds(winner.time)
        return Accepted(pixelBuffer: winner.pixelBuffer, time: winner.time,
                        stats: winner.stats, pose: winner.pose)
    }

    private func score(stats: FrameAnalysis.LumaStats) -> Double? {
        guard stats.laplacianVariance >= minSharpness else { return nil }
        guard stats.overexposedFraction <= maxOverexposed else { return nil }
        guard lumaRange.contains(stats.meanLuma) else { return nil }
        let sharp = min(stats.laplacianVariance / 400.0, 1.5)
        let exposure = 1.0 - abs(stats.meanLuma - 120.0) / 120.0
        let clipPenalty = stats.overexposedFraction * 2.5
        return sharp * 2.0 + exposure - clipPenalty
    }

    private func passesNovelty(_ cand: Candidate) -> Bool {
        // First keep is always allowed if it passed quality.
        guard lastAcceptedStats != nil else { return true }

        // Got a pose now but the first keep had none — treat as new.
        if lastAcceptedPose == nil {
            return cand.pose != nil
        }

        // No pose on this candidate: allow only after a longer gap and a
        // clear photometry change (avoid freezing capture when tracking dips).
        guard let pose = cand.pose, let prev = lastAcceptedPose else {
            guard let lastT = lastAcceptTime,
                  let last = lastAcceptedStats else { return true }
            let t = CMTimeGetSeconds(cand.time)
            guard t - lastT >= minIntervalSeconds * 2 else { return false }
            let dLap = abs(cand.stats.laplacianVariance - last.laplacianVariance)
            let dLuma = abs(cand.stats.meanLuma - last.meanLuma)
            return dLap > 40 || dLuma > 18
        }

        let p0 = SIMD3(prev.columns.3.x, prev.columns.3.y, prev.columns.3.z)
        let p1 = SIMD3(pose.columns.3.x, pose.columns.3.y, pose.columns.3.z)
        let moved = simd_distance(p0, p1)
        let f0 = -SIMD3(prev.columns.2.x, prev.columns.2.y, prev.columns.2.z)
        let f1 = -SIMD3(pose.columns.2.x, pose.columns.2.y, pose.columns.2.z)
        let turn = acos(min(1, max(-1, simd_dot(simd_normalize(f0),
                                                simd_normalize(f1)))))
        // Need a meaningful new viewpoint: move OR turn enough.
        return moved >= minMoveM || turn >= minTurnRad
    }

    private func copyBuffer(_ src: CVPixelBuffer) -> CVPixelBuffer? {
        let w = CVPixelBufferGetWidth(src)
        let h = CVPixelBufferGetHeight(src)
        let fmt = CVPixelBufferGetPixelFormatType(src)
        var dst: CVPixelBuffer?
        let status = CVPixelBufferCreate(nil, w, h, fmt, [
            kCVPixelBufferIOSurfacePropertiesKey as String: [:]
        ] as CFDictionary, &dst)
        guard status == kCVReturnSuccess, let dst else { return nil }
        CVPixelBufferLockBaseAddress(src, .readOnly)
        CVPixelBufferLockBaseAddress(dst, [])
        defer {
            CVPixelBufferUnlockBaseAddress(src, .readOnly)
            CVPixelBufferUnlockBaseAddress(dst, [])
        }
        let planes = CVPixelBufferGetPlaneCount(src)
        if planes == 0 {
            if let s = CVPixelBufferGetBaseAddress(src),
               let d = CVPixelBufferGetBaseAddress(dst) {
                memcpy(d, s, CVPixelBufferGetDataSize(src))
            }
        } else {
            for p in 0..<planes {
                guard let s = CVPixelBufferGetBaseAddressOfPlane(src, p),
                      let d = CVPixelBufferGetBaseAddressOfPlane(dst, p)
                else { continue }
                let rows = CVPixelBufferGetHeightOfPlane(src, p)
                let bytes = CVPixelBufferGetBytesPerRowOfPlane(src, p)
                let dstBytes = CVPixelBufferGetBytesPerRowOfPlane(dst, p)
                for r in 0..<rows {
                    memcpy(d.advanced(by: r * dstBytes),
                           s.advanced(by: r * bytes),
                           min(bytes, dstBytes))
                }
            }
        }
        return dst
    }
}

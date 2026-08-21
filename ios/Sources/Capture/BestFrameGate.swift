import CoreMedia
import CoreVideo
import Foundation
import simd

/// One-second windows: keep evaluating live frames, then accept at most the
/// single best that clears quality + novelty gates.
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

    var minSharpness: Double = 80
    var maxOverexposed: Double = 0.12
    var lumaRange: ClosedRange<Double> = 28...210
    var minMoveM: Float = 0.12
    var minTurnRad: Float = 0.12
    var windowSeconds: Double = 1.0

    private var windowStart: Double?
    private var best: Candidate?
    private var lastAcceptedPose: simd_float4x4?
    private var lastAcceptedStats: FrameAnalysis.LumaStats?

    func consider(pixelBuffer: CVPixelBuffer, time: CMTime,
                  pose: simd_float4x4?) -> Accepted? {
        let t = CMTimeGetSeconds(time)
        if windowStart == nil { windowStart = t }

        let stats = FrameAnalysis.lumaStats(of: pixelBuffer)
        if let scored = score(stats: stats),
           let retained = copyBuffer(pixelBuffer) {
            let cand = Candidate(pixelBuffer: retained, time: time,
                                 stats: stats, score: scored, pose: pose)
            if best == nil || cand.score > best!.score {
                best = cand
            }
        }

        guard let start = windowStart, t - start >= windowSeconds else { return nil }
        defer {
            windowStart = t
            best = nil
        }
        guard let winner = best else { return nil }
        guard passesNovelty(winner) else { return nil }

        lastAcceptedPose = winner.pose
        lastAcceptedStats = winner.stats
        return Accepted(pixelBuffer: winner.pixelBuffer, time: winner.time,
                        stats: winner.stats, pose: winner.pose)
    }

    private func score(stats: FrameAnalysis.LumaStats) -> Double? {
        guard stats.laplacianVariance >= minSharpness else { return nil }
        guard stats.overexposedFraction <= maxOverexposed else { return nil }
        guard lumaRange.contains(stats.meanLuma) else { return nil }
        let sharp = min(stats.laplacianVariance / 400.0, 1.5)
        let exposure = 1.0 - abs(stats.meanLuma - 120.0) / 120.0
        let clipPenalty = stats.overexposedFraction * 2.0
        return sharp * 2.0 + exposure - clipPenalty
    }

    private func passesNovelty(_ cand: Candidate) -> Bool {
        if let last = lastAcceptedStats {
            let dLap = abs(cand.stats.laplacianVariance - last.laplacianVariance)
            let dLuma = abs(cand.stats.meanLuma - last.meanLuma)
            if dLap < 15, dLuma < 6, cand.pose == nil, lastAcceptedPose == nil {
                return false
            }
        }
        guard let pose = cand.pose, let prev = lastAcceptedPose else {
            return true
        }
        let p0 = SIMD3(prev.columns.3.x, prev.columns.3.y, prev.columns.3.z)
        let p1 = SIMD3(pose.columns.3.x, pose.columns.3.y, pose.columns.3.z)
        let moved = simd_distance(p0, p1)
        let f0 = -SIMD3(prev.columns.2.x, prev.columns.2.y, prev.columns.2.z)
        let f1 = -SIMD3(pose.columns.2.x, pose.columns.2.y, pose.columns.2.z)
        let turn = acos(min(1, max(-1, simd_dot(simd_normalize(f0),
                                                simd_normalize(f1)))))
        if moved < minMoveM, turn < minTurnRad { return false }
        return true
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

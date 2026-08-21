import CoreMedia
import CoreVideo
import Foundation
import simd

/// Evaluates every preview frame; accepts at most one JPEG per second when
/// sharp enough and a new viewpoint. Candidates are stored as JPEG bytes so
/// max-res ultra-wide does not depend on a second full-size pixel-buffer copy
/// (those copies were failing silently → zero saves).
final class BestFrameGate {
    struct Candidate {
        var jpeg: Data
        var time: CMTime
        var stats: FrameAnalysis.LumaStats
        var score: Double
        var pose: simd_float4x4?
    }

    struct Accepted {
        var jpeg: Data
        var time: CMTime
        var stats: FrameAnalysis.LumaStats
        var pose: simd_float4x4?
    }

    var minIntervalSeconds: Double = 1.0
    var minMoveM: Float = 0.12
    var minTurnRad: Float = 0.14
    /// Ultra-wide is softer than 1×; keep this low.
    var minSharpness: Double = 18
    var maxOverexposed: Double = 0.22
    /// Video-range luma lives roughly 16…235.
    var lumaRange: ClosedRange<Double> = 18...230

    /// Encode the live buffer to JPEG when it becomes the window best.
    var encodeJPEG: ((CVPixelBuffer) -> Data?)?

    private(set) var lastSkipReason = "waiting"

    private var windowStart: Double?
    private var best: Candidate?
    private var lastAcceptedPose: simd_float4x4?
    private var lastAcceptedStats: FrameAnalysis.LumaStats?
    private var lastAcceptTime: Double?

    func reset() {
        windowStart = nil
        best = nil
        lastAcceptedPose = nil
        lastAcceptedStats = nil
        lastAcceptTime = nil
        lastSkipReason = "waiting"
    }

    func consider(pixelBuffer: CVPixelBuffer, time: CMTime,
                  pose: simd_float4x4?) -> Accepted? {
        let t = CMTimeGetSeconds(time)
        if windowStart == nil { windowStart = t }

        let stats = FrameAnalysis.lumaStats(of: pixelBuffer)
        if let scored = score(stats: stats) {
            let beats = best == nil
                || scored > best!.score
                || (pose != nil && best?.pose == nil && scored >= best!.score * 0.9)
            if beats {
                if let jpeg = encodeJPEG?(pixelBuffer) {
                    best = Candidate(jpeg: jpeg, time: time,
                                     stats: stats, score: scored, pose: pose)
                    lastSkipReason = "holding candidate"
                } else {
                    lastSkipReason = "JPEG encode failed"
                }
            }
        } else {
            lastSkipReason = rejectReason(stats)
        }

        guard let start = windowStart,
              t - start >= minIntervalSeconds else { return nil }

        let winner = best
        windowStart = t
        best = nil

        // Soft fallback: if nothing passed the sharp gate, still keep a usable
        // exposure frame so a scan is never stuck at 0 photos.
        let chosen: Candidate?
        if let winner {
            chosen = winner
        } else if lumaRange.contains(stats.meanLuma),
                  stats.overexposedFraction <= maxOverexposed,
                  let jpeg = encodeJPEG?(pixelBuffer) {
            chosen = Candidate(jpeg: jpeg, time: time, stats: stats,
                               score: 0, pose: pose)
            lastSkipReason = "soft keep (low sharpness)"
        } else {
            lastSkipReason = rejectReason(stats)
            return nil
        }

        guard let chosen else { return nil }

        if let lastT = lastAcceptTime, t - lastT < minIntervalSeconds {
            lastSkipReason = "rate limit"
            return nil
        }
        guard passesNovelty(chosen) else {
            lastSkipReason = "need new viewpoint"
            return nil
        }

        lastAcceptedPose = chosen.pose
        lastAcceptedStats = chosen.stats
        lastAcceptTime = CMTimeGetSeconds(chosen.time)
        lastSkipReason = "saved"
        return Accepted(jpeg: chosen.jpeg, time: chosen.time,
                        stats: chosen.stats, pose: chosen.pose)
    }

    private func score(stats: FrameAnalysis.LumaStats) -> Double? {
        guard stats.laplacianVariance >= minSharpness else { return nil }
        guard stats.overexposedFraction <= maxOverexposed else { return nil }
        guard lumaRange.contains(stats.meanLuma) else { return nil }
        let sharp = min(stats.laplacianVariance / 200.0, 1.5)
        let exposure = 1.0 - abs(stats.meanLuma - 110.0) / 110.0
        let clipPenalty = stats.overexposedFraction * 2.5
        return sharp * 2.0 + exposure - clipPenalty
    }

    private func rejectReason(_ stats: FrameAnalysis.LumaStats) -> String {
        if stats.laplacianVariance < minSharpness {
            return String(format: "blurry (%.0f)", stats.laplacianVariance)
        }
        if stats.overexposedFraction > maxOverexposed {
            return String(format: "blown highlights (%.0f%%)",
                          stats.overexposedFraction * 100)
        }
        if stats.meanLuma < lumaRange.lowerBound {
            return String(format: "too dark (%.0f)", stats.meanLuma)
        }
        if stats.meanLuma > lumaRange.upperBound {
            return String(format: "too bright (%.0f)", stats.meanLuma)
        }
        return "waiting"
    }

    private func passesNovelty(_ cand: Candidate) -> Bool {
        guard lastAcceptedStats != nil else { return true }
        if lastAcceptedPose == nil {
            return true
        }
        guard let pose = cand.pose, let prev = lastAcceptedPose else {
            guard let lastT = lastAcceptTime,
                  let last = lastAcceptedStats else { return true }
            let t = CMTimeGetSeconds(cand.time)
            guard t - lastT >= minIntervalSeconds * 1.5 else { return false }
            let dLap = abs(cand.stats.laplacianVariance - last.laplacianVariance)
            let dLuma = abs(cand.stats.meanLuma - last.meanLuma)
            return dLap > 25 || dLuma > 12
        }

        let p0 = SIMD3(prev.columns.3.x, prev.columns.3.y, prev.columns.3.z)
        let p1 = SIMD3(pose.columns.3.x, pose.columns.3.y, pose.columns.3.z)
        let moved = simd_distance(p0, p1)
        let f0 = -SIMD3(prev.columns.2.x, prev.columns.2.y, prev.columns.2.z)
        let f1 = -SIMD3(pose.columns.2.x, pose.columns.2.y, pose.columns.2.z)
        let turn = acos(min(1, max(-1, simd_dot(simd_normalize(f0),
                                                simd_normalize(f1)))))
        return moved >= minMoveM || turn >= minTurnRad
    }
}

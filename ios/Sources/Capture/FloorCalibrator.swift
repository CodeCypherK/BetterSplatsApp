import Foundation

/// Measuring the floor at the start of a capture, so the reconstruction can
/// be levelled against a fact instead of an inference.
///
/// Working out which plane is the floor after the fact is genuinely hard —
/// a ceiling satisfies every geometric test a floor does, a scan holding one
/// big plane is ambiguous outright, and floors are sparsely tracked because
/// they are low on texture and seen at a glancing angle. Pointing the phone
/// at the floor for a moment answers all of it: the user knows which surface
/// it is, and the depth sensor sees a dense sheet of it from a metre away.
///
/// Two things this has to get right, both learned by measuring the pipeline
/// rather than by reasoning about it:
///
/// 1. The calibration is recorded against a frame that is **actually stored
///    in the session**. It holds the plane in that frame's camera
///    coordinates, so the solve turns it into a world plane using that
///    frame's final pose. Filed against a frame the solve never registered,
///    or against a pose that was never really captured, it puts the floor
///    metres out — silently, because nothing about it looks wrong.
///
/// 2. The user has to be **moving**. Sweeping the phone up while standing
///    still is pure rotation: nothing triangulates, the solve registers none
///    of those frames, and the calibration is quietly discarded. That is why
///    the prompt asks for a step forward rather than just an aim.
@MainActor
final class FloorCalibrator {
    enum Phase: Equatable {
        /// Looking for the floor. `advice` comes from the engine so the
        /// wording cannot drift between the two.
        case aiming(advice: String, heightM: Double?)
        /// Found and recorded; the user is being asked to start scanning.
        case measured(heightM: Double)
        /// The user skipped it, or it never converged. The solve falls back
        /// to inferring the floor, which still works — just less well.
        case skipped
    }

    private(set) var phase: Phase = .aiming(advice: "Point at the floor", heightM: nil)

    /// A reading has to hold up across several frames before it is accepted.
    /// One good frame can be a coincidence of where the phone happened to be
    /// pointing mid-swing; a second of them is a decision.
    private static let framesToConfirm = 8
    /// Frames are offered from the STORED stream, which is already thinned to
    /// about 3 fps — so every offered frame gets fitted. This was decimating
    /// by 3 on the assumption of a 30 fps feed, which stretched the eight
    /// confirmations to roughly seven seconds of holding the phone at the
    /// floor. At the stored rate it is about two and a half.
    private static let frameInterval = 1

    /// Frames offered before giving up and letting the session get on with
    /// it. Someone in a room where the fit will never converge — deep pile
    /// carpet, a mirror floor, a cluttered workshop — should not be held at a
    /// prompt that cannot be satisfied. Inference still levels the scan.
    private static let framesBeforeGivingUp = 60

    private var consecutiveGood = 0
    private var frameCounter = 0
    private(set) var isComplete = false

    /// Result of offering one frame, for the caller to persist.
    struct Accepted {
        let frameId: UInt32
        let normal: (Double, Double, Double)
        let heightM: Double
        let rmseM: Double
        let incidenceDeg: Double
        let inliers: Int32
    }

    /// Offers one depth frame. `storedFrameId` must be the id of a frame the
    /// session has actually written — pass nil when this frame was not
    /// stored, and the reading still updates the prompt but cannot be
    /// accepted against it.
    func offer(depthF32: [Float], width: Int32, height: Int32,
               fx: Double, fy: Double, cx: Double, cy: Double,
               storedFrameId: UInt32?) -> Accepted? {
        guard !isComplete else { return nil }

        frameCounter += 1
        if frameCounter >= Self.framesBeforeGivingUp {
            skip()
            return nil
        }
        guard frameCounter % Self.frameInterval == 0 else { return nil }

        let plane = depthF32.withUnsafeBufferPointer { buffer -> CoreEngine.FloorPlane? in
            guard let base = buffer.baseAddress else { return nil }
            return CoreEngine.fitFloorPlane(depth: base, width: width,
                                            height: height, fx: fx, fy: fy,
                                            cx: cx, cy: cy)
        }
        guard let plane else {
            phase = .aiming(advice: "Point at the floor", heightM: nil)
            return nil
        }

        guard plane.isUsable else {
            consecutiveGood = 0
            phase = .aiming(advice: plane.advice, heightM: nil)
            return nil
        }

        consecutiveGood += 1
        phase = .aiming(advice: plane.advice, heightM: plane.heightM)

        // Only a stored frame can carry the calibration: the solve resolves
        // it through that frame's pose, and a frame that was never written
        // has no pose to resolve through.
        guard consecutiveGood >= Self.framesToConfirm,
              let frameId = storedFrameId else { return nil }

        isComplete = true
        phase = .measured(heightM: plane.heightM)
        return Accepted(frameId: frameId, normal: plane.normal,
                        heightM: plane.heightM, rmseM: plane.rmseM,
                        incidenceDeg: plane.incidenceDeg,
                        inliers: plane.inliers)
    }

    /// The user chose to get on with it. Levelling falls back to inferring
    /// the floor from the reconstruction, which is a real capability and not
    /// a failure — just a less certain one.
    func skip() {
        guard !isComplete else { return }
        isComplete = true
        phase = .skipped
    }
}

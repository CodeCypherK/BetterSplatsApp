import Foundation
import simd

/// Where the user is standing, and how to talk about a place relative to it.
///
/// The readiness grid works in the session's world frame, whose origin is
/// wherever the first keyframe happened to land. That frame is right for the
/// engine and useless for talking to a person: "3.2 m away" measured from it
/// means "3.2 m from where you started", which is not what anyone reads it
/// as. Everything the user is told about a place therefore goes through the
/// live pose first.
struct ViewerPose {
    /// World-to-camera, COLMAP convention: x_cam = R(q) * x_world + t.
    let rotation: simd_quatd
    let translation: SIMD3<Double>

    init?(status: bs_live_status) {
        guard status.pose_valid != 0 else { return nil }
        // bs_api gives (w, x, y, z); simd_quatd takes (ix, iy, iz, r).
        rotation = simd_quatd(ix: status.q.1, iy: status.q.2,
                              iz: status.q.3, r: status.q.0)
        translation = SIMD3(status.t.0, status.t.1, status.t.2)
    }

    /// The camera centre in world coordinates: C = -R^T t.
    var center: SIMD3<Double> {
        -(rotation.inverse.act(translation))
    }

    func distance(to worldPoint: SIMD3<Float>) -> Double {
        simd_distance(center, Self.widened(worldPoint))
    }

    /// A world direction expressed in the camera's frame. +x is the user's
    /// right, +y is down, +z is where they are looking.
    func inCameraFrame(_ worldDirection: SIMD3<Float>) -> SIMD3<Double> {
        rotation.act(Self.widened(worldDirection))
    }

    private static func widened(_ v: SIMD3<Float>) -> SIMD3<Double> {
        SIMD3<Double>(Double(v.x), Double(v.y), Double(v.z))
    }
}

/// Which way to send someone, in the only vocabulary that survives being
/// read at arm's length while walking: left, right, forward, back.
enum MoveBearing: String {
    case left, right, forward, back

    /// From a world-space direction and the viewer's orientation. Vertical
    /// components are dropped — the instruction is for someone's feet.
    init(worldDirection: SIMD3<Float>, viewer: ViewerPose) {
        let local = viewer.inCameraFrame(worldDirection)
        if abs(local.x) >= abs(local.z) {
            self = local.x >= 0 ? .right : .left
        } else {
            self = local.z >= 0 ? .forward : .back
        }
    }
}

/// What to tell the user about one weak area.
///
/// Every string here is an instruction for a body, not a description of a
/// deficit. The rule the wording follows: name the action first, in words
/// that need no vocabulary from this project, and let the reason follow only
/// where it changes what the user would do. "Insufficient visual geometry"
/// tells someone holding a phone nothing they can act on; "step left and
/// scan it again slowly" tells them everything.
enum WeakAreaGuidance {
    /// The five sub-score axes, in ABI order.
    static let axisNames = ["Geometry", "Camera poses", "Texture",
                            "LiDAR coverage", "View overlap"]

    static func surfaceName(kind: Int, side: Int) -> String {
        guard kind == 0 else {
            return ["Wall", "Floor", "Ceiling", "Object"][min(3, max(0, kind))]
        }
        let placement = ["", "Back ", "Left ", "Right ", "Front "][
            min(4, max(0, side))]
        return "\(placement)wall"
    }

    /// Heading for one weak area. The distance is from the USER when the
    /// tracker knows where they are, and omitted entirely when it does not —
    /// a number measured from the wrong origin is worse than no number.
    static func title(for area: CoreEngine.Snapshot.WeakArea,
                      viewer: ViewerPose?, regionName: String? = nil) -> String {
        var base = surfaceName(kind: area.surfaceKind, side: area.surfaceSide)
        if let viewer {
            base += String(format: " · %.1f m away", viewer.distance(to: area.center))
        }
        if let regionName { return "\(regionName) — \(base)" }
        return base
    }

    static func advice(for area: CoreEngine.Snapshot.WeakArea,
                       viewer: ViewerPose?) -> String {
        let feet = max(1, Int((Double(area.moveDistM) * 3.28).rounded()))
        // Without a pose there is no left or right, so the wording falls back
        // to a direction anyone can follow from where they stand.
        let bearing = viewer.map {
            MoveBearing(worldDirection: area.moveDir, viewer: $0).rawValue
        }

        switch area.deficiency {
        case 0:  // geometry: too few well-triangulated points on the surface
            if let bearing {
                return "Step \(feet) ft to the \(bearing), then scan this "
                     + "surface again slowly."
            }
            return "Step \(feet) ft to one side, then scan this surface again "
                 + "slowly."

        case 1:  // camera poses: the frames that saw it are poorly constrained
            return "The camera positions here are shaky. Walk back to an area "
                 + "you have already scanned well, then come at this again "
                 + "without turning quickly."

        case 2:  // texture: LiDAR carrying it, image detail too coarse
            return "Get closer — about an arm's length or two — and pan across "
                 + "it slowly. Right now the shape is right but the detail is "
                 + "soft."

        case 3:  // LiDAR coverage: out of range or too oblique
            return "Move within 10 ft and face it straight on. At a glancing "
                 + "angle the depth sensor barely reads this surface."

        default:  // view overlap: seen from too narrow a cone of directions
            if let bearing {
                return "Walk a slow curve past it, starting \(feet) ft to the "
                     + "\(bearing), keeping it in frame the whole way."
            }
            return "Walk a slow curve past it, keeping it in frame the whole "
                 + "way. It has only been seen from one direction."
        }
    }
}

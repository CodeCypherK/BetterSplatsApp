#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "common/geometry.h"
#include "io/depth_codec.h"

namespace bs::synth {

// A textured rectangle in world space: corner + two edge vectors. The
// procedural texture is world-anchored (sampled in plane-local meters), so
// multi-view appearance is consistent — SIFT/ORB find real correspondences.
struct TexturedPlane {
  Eigen::Vector3d origin;
  Eigen::Vector3d edge_u;  // full extent along u, meters
  Eigen::Vector3d edge_v;  // full extent along v, meters
  cv::Vec3f base_color{0.72f, 0.72f, 0.72f};  // BGR in [0,1]
  float texture_amount = 0.5f;  // 0 = perfectly blank surface
  uint32_t texture_seed = 1;
};

struct Scene {
  std::vector<TexturedPlane> planes;
};

struct RayHit {
  double t = -1.0;          // camera-space depth (z), <0 = miss
  int plane_index = -1;
  double u = 0, v = 0;      // hit position, plane-local meters
};

// World frame: Y up, floor at y=0. Standard CV camera: +Z forward, +Y down.
// A furnished room with two well-textured walls, one nearly blank wall,
// floor/ceiling, and two interior boxes.
Scene MakeRoomScene(uint32_t seed, bool blank_wall = true);

struct RoomBounds {
  double x0, x1, z0, z1;
  double width() const { return x1 - x0; }
  double depth() const { return z1 - z0; }
  // The room's scale for capture planning: the SHORT side. Orbits are flown
  // at half of this, and half the long side of a corridor puts the camera
  // through a wall.
  double scale() const { return std::min(width(), depth()); }
  Eigen::Vector3d centre(double y) const {
    return {0.5 * (x0 + x1), y, 0.5 * (z0 + z1)};
  }
};

// One piece of furniture: a box standing on the floor. Large enough to be
// worth walking around, which makes it an orbit target for the capture plan.
struct FurnitureBox {
  Eigen::Vector3d origin;  // minimum corner, y = floor
  double w, h, d;
  cv::Vec3f color;
  float texture;
  uint32_t seed_xor;
  Eigen::Vector3d centre() const {
    return {origin.x() + w / 2, origin.y() + h / 2, origin.z() + d / 2};
  }
};

// The two-room layout in one place. The scene builder walls it in and the
// trajectories walk it; keeping two copies of these numbers is how a capture
// path ends up strolling through a divider that moved 20 cm.
struct TwoRoomLayout {
  double height = 2.5;
  RoomBounds a{-3.0, 3.0, -4.0, 4.0};  // room A
  RoomBounds b{3.0, 9.0, -4.0, 4.0};   // room B, sharing the wall at x = 3
  double door_x = 3.0;                 // centre plane of the divider
  double door_half = 0.8;              // 1.6 m cased opening
  double door_height = 2.1;
  // The divider is a real wall with two faces, so the opening has jambs and
  // a soffit — surfaces that only exist because the wall has depth, and the
  // reason orbiting a doorway returns anything at all.
  double wall_thickness = 0.16;
  std::vector<FurnitureBox> furniture;  // room A first, then room B

  double face_a() const { return door_x - wall_thickness / 2; }
  double face_b() const { return door_x + wall_thickness / 2; }
  // Middle of the opening, at the height a person's eye crosses it.
  Eigen::Vector3d door_centre() const {
    return {door_x, door_height * 0.5, 0.0};
  }
};
const TwoRoomLayout& TwoRoomLayoutSpec();

// Two rooms sharing a dividing wall with an open doorway, per
// TwoRoomLayoutSpec(). Exercises multi-room region clustering and the drift
// that accumulates walking between spaces.
Scene MakeTwoRoomScene(uint32_t seed, bool blank_wall = true);

// Nearest intersection of the pixel ray with the scene. `dir_world` must be
// R_cw * (x_n, y_n, 1) — unnormalized, so `t` is camera-space depth.
RayHit CastRay(const Scene& scene, const Eigen::Vector3d& center,
               const Eigen::Vector3d& dir_world);

// Procedural texture value in [0,1] at plane-local meters (u,v).
float TextureValue(const TexturedPlane& plane, double u, double v);

// Photometric/optical realism knobs for RenderImage. The defaults reproduce
// the original clean render exactly (mild blur + light sensor noise), so
// existing goldens are unaffected unless a knob is changed.
struct RenderOptions {
  float gain = 1.0f;            // multiplicative exposure/gain (AE drift)
  float noise_sigma = 1.6f;     // RGB sensor noise stddev, DN
  float blur_sigma = 0.6f;      // isotropic optical blur, px
  float motion_blur_px = 0.0f;  // linear motion-blur length, px (0 = none)
  double motion_blur_angle = 0.0;  // motion-blur direction, radians
};

// Renders a BGR uint8 image of the scene from `pose` (world-to-camera).
cv::Mat RenderImage(const Scene& scene, const SE3& pose, const Intrinsics& K,
                    const RenderOptions& opts = {});

struct DepthNoise {
  double sigma_base_m = 0.004;
  double sigma_quadratic = 0.0035;  // sigma = base + q * d^2
  double dropout_frac = 0.01;
  double max_range_m = 5.0;
  double grazing_dropout_cos = 0.25;  // drop when |cos(incidence)| below this
};

// Renders a metric float16 depth map with LiDAR-like noise. Deterministic
// for a given (seed, frame) pair.
DepthImage RenderDepth(const Scene& scene, const SE3& pose, const Intrinsics& K,
                       const DepthNoise& noise, uint32_t seed);

// Smooth orbit inside the room: positions on an ellipse at eye height,
// looking outward, with mild handheld-style height/heading jitter.
// `sweep_deg` is the total orbit arc across the whole sequence — keep the
// per-frame angular change realistic (a phone at 30 fps moves well under
// 1 deg/frame; stored-frame sequences a few deg/frame). Returns
// world-to-camera poses.
// `max_turn_deg` caps how far the view may swing between consecutive frames.
// It is what makes a declared rotation rate binding: a trajectory expresses
// where it wants to look, and where that intent moves quickly (rounding a
// corner, crossing a doorway) the camera follows at a rate a wrist can
// produce instead of snapping. 0 keeps a permissive default.
std::vector<SE3> OrbitTrajectory(int frame_count, double radius_x, double radius_z,
                                 double eye_height, uint32_t seed,
                                 double sweep_deg = 140.0,
                                 double max_turn_deg = 0.0);

// What the camera is doing at each point of the capture walk. The plan is
// worth being able to LOOK at — a path that reads correctly in numbers can
// still be walking somewhere absurd, and the bug that cost the most this
// far (a lap pointed square at a wall half a metre away) was obvious the
// moment the view directions were drawn and invisible in every summary
// statistic before that.
enum class CapturePhase : uint8_t {
  kLap,             // circling the room
  kOrbitObject,     // going round a piece of furniture
  kOrbitDoorway,    // going round the opening, from one room's side
  kThroughDoorway,  // crossing the threshold
  kApproach,        // walking between the above
};

// The capture walk before it is sampled into frames: a dense polyline with
// the look target and phase at every point. `CaptureTrajectory` resamples
// this at constant speed; tools render it.
struct CapturePlan {
  std::vector<Eigen::Vector3d> position;
  std::vector<Eigen::Vector3d> look;  // absolute world-space look target
  std::vector<CapturePhase> phase;
};
CapturePlan BuildCapturePlan(double eye_height);

// The capture walk through MakeTwoRoomScene: for each room, circle the room
// once, then orbit every large object in it — and the doorway is an object
// like any other, orbited from both rooms, because an opening is where two
// spaces have to agree and it is the one place a splat shows the seam.
//
// Orbits are flown at half the room's short dimension, which is the distance
// that keeps an object's whole surface in frame with the room behind it for
// context. Where a wall or another object is in the way the ring is cut to
// the arcs that are actually walkable and the camera steps around the gap
// still looking at the object, which is what a person does.
//
// The walk closes: it ends at the viewpoint it started from, so the sequence
// contains a genuine revisit for loop closure to find and for drift to be
// measured against.
std::vector<SE3> CaptureTrajectory(int frame_count, double eye_height,
                                   uint32_t seed, double max_turn_deg = 0.0);

// The optional opening circuit: one fast lap of both rooms hugging the
// walls with the camera aimed INWARD across each space. Deliberately unlike
// the capture walk — it maximizes how much of the room each frame sees and
// how far apart the views are, which is what a localization scaffold needs
// and the opposite of what splat detail needs. Never reconstructed from.
std::vector<SE3> ScoutTrajectory(int frame_count, double eye_height,
                                 uint32_t seed, double max_turn_deg = 0.0);

// How much a trajectory actually moves. Every trajectory above covers a
// FIXED physical path, so the frame count alone decides how fast the camera
// travels — and a count chosen for render cost rather than for physics
// silently produces motion no hand can make and no tracker can follow.
struct TrajectoryMotion {
  double path_m = 0;          // total distance travelled by the camera centre
  double turn_deg = 0;        // total rotation swept
  double mean_step_m = 0;     // per-frame translation
  double mean_turn_deg = 0;   // per-frame rotation
  double p95_step_m = 0;
  double p95_turn_deg = 0;
  // The worst single frame. This is the number that matters for tracking and
  // the one summary statistics hide: a look target that switches rooms as you
  // cross a doorway produced a single 174 deg frame in an otherwise clean
  // 0.54 deg/frame circuit, and that one frame ended tracking for the
  // remaining 800.
  double max_step_m = 0;
  double max_turn_deg = 0;
};
TrajectoryMotion MeasureMotion(const std::vector<SE3>& poses);

// How many frames the app's storage gate would keep along this path.
//
// The number that decides whether a capture plan is usable is not its length
// in metres but how many stored images it asks for: the target is 200-500 per
// room, above which a house does not fit in one project and below which the
// final solve runs out of baseline. A plan is only "done" once this has been
// counted against the same thresholds the engine ships with
// (EngineConfig::store_min_translation_m / store_min_rotation_deg).
// `depth_frac` and `scene` model the engine's depth-scaled spacing: a frame
// is due after max(min_step_m, depth_frac * subject distance), where the
// subject distance is what the optical axis actually hits. Omit them for the
// flat-distance behaviour.
int GatedFrameCount(const std::vector<SE3>& poses, double min_step_m,
                    double min_turn_deg, double depth_frac = 0.0,
                    const Scene* scene = nullptr);

// True when the camera centre is inside walkable floor: within a room, clear
// of the walls, out of the furniture, and — at the divider — inside the
// doorway rather than in the wall. `clearance` is how far from a solid the
// point must stay, so tests can ask the strict question ("did the path pass
// through a wall") separately from the planning question ("would a person
// walk here").
bool TwoRoomWalkable(const Eigen::Vector3d& p, double wall_clearance,
                     double object_clearance);

}  // namespace bs::synth

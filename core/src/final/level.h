#pragma once

#include <vector>

#include "common/geometry.h"

namespace bs {

// Putting the reconstruction the right way up.
//
// Image-only structure from motion has no idea which way is down. The world
// frame it produces is anchored on whatever the first camera happened to be
// doing, so a scan of a perfectly ordinary room routinely comes out with the
// floor on a slope and the walls at some arbitrary bearing. Nothing is wrong
// with it geometrically — every camera and point is self-consistent — but it
// is unpleasant to work with downstream, and it makes two scans of adjacent
// spaces needlessly hard to compare.
//
// This finds the floor and levels it, then finds the building's dominant
// wall direction and squares it to the axes. The result is a single rigid
// transform applied to everything at once, so all internal geometry is
// preserved exactly: it changes where the world sits, never its shape.
//
// The floor is identified by physics rather than by size alone. Walls are
// large planes too, but a wall always buries a good part of the scene on its
// far side, and a floor never does — nothing lives under a floor.
//
// The ceiling is the hard one, and it is worth being explicit about why.
// Orient a ceiling's normal toward the cameras and it satisfies every test a
// floor does: the cameras are all on its positive side, nothing is beyond
// it, and it sits a plausible room-height away. In a symmetric room it even
// ties on inlier count. The two are told apart by the one asymmetry a
// hand-held capture always has — the phone is carried above the mid-height
// of the room, roughly 1.5 m up in a 2.4-3.0 m space — so of the two
// opposed extremal planes, the floor is the FARTHER. That assumption is
// stated here rather than buried: a capture made with the phone near the
// floor, or a ceiling under about 2 m, would break it, and in those cases
// leaving the scan as solved is the better outcome.

struct LevelingOptions {
  double floor_inlier_m = 0.06;      // plane inlier band
  double min_camera_height_m = 0.7;  // floor sits at least this far below
  double max_camera_height_m = 3.0;  // ...and no further than this
  int min_floor_inliers = 80;
  int ransac_iterations = 24000;
  // The floor is usually a small share of the tracked points — floors are
  // low on texture and seen at a glancing angle — so sampling three points
  // blind almost never lands three of them on it. The camera trajectory
  // supplies a prior instead: a phone carried around a room stays at
  // roughly one height, so the plane through the camera centres is
  // horizontal and its normal is up to within a few degrees. Candidates
  // whose normal is further than this from that estimate are discarded
  // before the expensive inlier count. Ignored when the trajectory is too
  // straight to define a plane.
  double prior_max_angle_deg = 35.0;
  double prior_min_planarity = 0.02;
  // Fraction of the reconstruction allowed to sit BELOW the candidate. This
  // is the test that actually finds a floor: a plane tilted through the room
  // catches plenty of wall points and keeps the cameras above it, but it
  // always buries a large share of the scene underneath. A real floor has
  // almost nothing under it.
  double max_below_frac = 0.02;
  // Two opposed extremal planes have to differ by at least this much before
  // the farther one is taken as the floor. Below it they are too alike to
  // call, and the scan is left as solved.
  double min_floor_ceiling_gap_m = 0.25;

  // Squaring the walls to the axes. Points in this band above the levelled
  // floor are wall candidates: high enough to clear furniture and clutter,
  // low enough to stay below the ceiling.
  bool align_walls = true;
  double wall_band_lo_m = 0.4;
  double wall_band_hi_m = 2.2;
  double wall_bin_m = 0.10;      // histogram bin for the Manhattan search
  double wall_step_deg = 0.25;   // angular resolution of that search
  int min_wall_points = 60;
};

struct Leveling {
  bool floor_found = false;
  // True when the floor came from a capture-time calibration rather than
  // from searching the reconstruction. Worth reporting: the two have very
  // different failure modes.
  bool floor_measured = false;
  bool walls_squared = false;
  // world_levelled = transform.Apply(world_raw). Rigid: rotation + offset.
  SE3 transform;

  int floor_inliers = 0;
  double floor_rmse_m = 0;
  // How far the world was rotated. NOT a quality signal: an image-only solve
  // anchors its frame on whatever the first camera was doing, so this is
  // routinely large — 168 deg on a perfectly good scan that happened to
  // start upside down — and says nothing about the reconstruction.
  double rotation_deg = 0;
  double wall_turn_deg = 0;

  // These two ARE quality signals, and cheap ones. A hand-held walk holds
  // the phone at close to one height, so after levelling the camera heights
  // should cluster tightly around eye level. A wide spread means the floor
  // fit is fighting a warped reconstruction, and the levelling is only as
  // good as the geometry underneath it.
  double camera_height_m = 0;
  double camera_height_spread_m = 0;
};

// Builds the transform from a floor that was MEASURED at capture time
// rather than inferred: the plane as the depth sensor saw it, in the
// calibration frame's camera coordinates, plus that frame's final pose.
//
// This is strictly better than inference when it is available. Every
// ambiguity the search has to reason around — floor against ceiling, a lone
// plane, a floor too sparsely tracked to find — was settled at capture time
// by a person pointing the phone at the floor, and the depth sensor saw a
// dense sheet of it from a metre away instead of a scattering of features
// at a glancing angle. Walls are still squared from the geometry, since
// that part has no ambiguity once the floor is known.
Leveling LevelingFromMeasuredFloor(const Eigen::Vector3d& camera_normal,
                                   double camera_offset_m,
                                   const SE3& calibration_pose,
                                   const std::vector<Eigen::Vector3d>& points,
                                   const std::vector<Eigen::Vector3d>& camera_centres,
                                   const LevelingOptions& options = {});

// Estimates the transform from a reconstruction's points and camera centres.
// Returns floor_found = false and an identity transform when no plane fits
// the description of a floor — better to leave a scan tilted than to level
// it against a table top.
Leveling EstimateLeveling(const std::vector<Eigen::Vector3d>& points,
                          const std::vector<Eigen::Vector3d>& camera_centres,
                          const LevelingOptions& options = {});

}  // namespace bs

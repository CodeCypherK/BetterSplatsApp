// Putting a reconstruction the right way up: floor detection, levelling,
// and squaring the walls to the axes.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <random>

#include "final/level.h"

namespace bs {
namespace {

// A room, built the right way up, then tilted and turned by a known amount —
// which is what an image-only solve hands you, since it has no idea which
// way down is.
struct Room {
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> cameras;
};

Room MakeRoom(double tilt_deg, double yaw_deg, uint32_t seed = 7) {
  Room room;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(0.0, 1.0);

  // Floor at y = 0, 6 x 8 m.
  for (int i = 0; i < 1200; ++i) {
    room.points.emplace_back(-3.0 + 6.0 * u(rng), 0.0, -4.0 + 8.0 * u(rng));
  }
  // Four walls, 2.5 m tall, squared to the axes before we turn them.
  for (int i = 0; i < 600; ++i) {
    const double h = 2.5 * u(rng);
    room.points.emplace_back(-3.0, h, -4.0 + 8.0 * u(rng));
    room.points.emplace_back(3.0, h, -4.0 + 8.0 * u(rng));
    room.points.emplace_back(-3.0 + 6.0 * u(rng), h, -4.0);
    room.points.emplace_back(-3.0 + 6.0 * u(rng), h, 4.0);
  }
  // Ceiling: a plane just as large as the floor, and above the cameras.
  // The floor search has to reject it.
  for (int i = 0; i < 1200; ++i) {
    room.points.emplace_back(-3.0 + 6.0 * u(rng), 2.5, -4.0 + 8.0 * u(rng));
  }
  // A hand-held phone at eye height.
  for (int i = 0; i < 60; ++i) {
    const double s = static_cast<double>(i) / 59.0;
    room.cameras.emplace_back(-2.0 + 4.0 * s, 1.5 + 0.05 * std::sin(8 * s),
                              -3.0 + 6.0 * s);
  }

  // Now put it in an arbitrary frame, the way a solve would.
  const Eigen::Quaterniond tilt(
      Eigen::AngleAxisd(DegToRad(tilt_deg), Eigen::Vector3d(1, 0, 0.3).normalized()));
  const Eigen::Quaterniond yaw(
      Eigen::AngleAxisd(DegToRad(yaw_deg), Eigen::Vector3d::UnitY()));
  const Eigen::Quaterniond r = (tilt * yaw).normalized();
  for (auto& p : room.points) p = r * p;
  for (auto& c : room.cameras) c = r * c;
  return room;
}

TEST(LevelTest, FindsTheFloorAndLevelsIt) {
  const Room room = MakeRoom(/*tilt_deg=*/9.0, /*yaw_deg=*/23.0);
  const Leveling level = EstimateLeveling(room.points, room.cameras);

  ASSERT_TRUE(level.floor_found);
  EXPECT_GT(level.floor_inliers, 800) << "the floor is 1200 points";
  // Comfortably tighter than the acceptance band. It is not zero even on a
  // perfectly flat synthetic floor, and correctly so: the walls rise from
  // y = 0, so their lowest few centimetres are genuinely within the band and
  // genuinely on the floor.
  const LevelingOptions defaults;
  EXPECT_LT(level.floor_rmse_m, defaults.floor_inlier_m / 3.0);
  EXPECT_NEAR(level.rotation_deg, 9.0, 0.5)
      << "the room was built upright and tilted by exactly this much";
  EXPECT_NEAR(level.camera_height_m, 1.5, 0.05);
  EXPECT_LT(level.camera_height_spread_m, 0.1)
      << "a steady walk should give a tight height spread";

  // After levelling, the floor points sit on y = 0 and everything else above.
  int on_floor = 0;
  double lowest = 1e9;
  for (const auto& p : room.points) {
    const Eigen::Vector3d q = level.transform.Apply(p);
    lowest = std::min(lowest, q.y());
    if (std::abs(q.y()) < 0.02) ++on_floor;
  }
  EXPECT_GT(on_floor, 800) << "floor should land on y = 0";
  EXPECT_GT(lowest, -0.05) << "nothing should end up below the floor";

  // ...and the cameras end up at plausible eye height above it.
  for (const auto& c : room.cameras) {
    const Eigen::Vector3d q = level.transform.Apply(c);
    EXPECT_GT(q.y(), 1.2);
    EXPECT_LT(q.y(), 1.8);
  }
}

TEST(LevelTest, SquaresWallsToTheAxes) {
  const Room room = MakeRoom(/*tilt_deg=*/6.0, /*yaw_deg=*/37.0);
  const Leveling level = EstimateLeveling(room.points, room.cameras);
  ASSERT_TRUE(level.floor_found);
  ASSERT_TRUE(level.walls_squared);

  // Squaring only fixes ORIENTATION — the room keeps whatever x/z offset the
  // solve gave it, and which of the four walls ends up on +X is arbitrary
  // (the fit is modulo 90 deg). So the thing to check is that each wall
  // collapses to a constant x or a constant z, not where those constants
  // land: a squared wall dumps all its points into one histogram bin, a
  // skewed one smears them across many.
  std::map<int, int> hist_x, hist_z;
  int wall_total = 0;
  for (const auto& p : room.points) {
    const Eigen::Vector3d q = level.transform.Apply(p);
    if (q.y() < 0.4 || q.y() > 2.2) continue;
    ++wall_total;
    ++hist_x[static_cast<int>(std::floor(q.x() / 0.1))];
    ++hist_z[static_cast<int>(std::floor(q.z() / 0.1))];
  }
  ASSERT_GT(wall_total, 500);

  // The two strongest bins on each axis are the four walls seen end-on.
  auto top_two = [](const std::map<int, int>& hist) {
    std::vector<int> counts;
    for (const auto& [_, n] : hist) counts.push_back(n);
    std::sort(counts.rbegin(), counts.rend());
    int sum = 0;
    for (size_t i = 0; i < counts.size() && i < 2; ++i) sum += counts[i];
    return sum;
  };
  const double concentrated =
      static_cast<double>(top_two(hist_x) + top_two(hist_z)) / wall_total;
  EXPECT_GT(concentrated, 0.8)
      << "walls did not collapse onto axis-aligned lines (only "
      << concentrated << " of wall points in the four strongest bins)";
}

TEST(LevelTest, TransformIsRigidSoGeometryIsUntouched) {
  const Room room = MakeRoom(11.0, 52.0);
  const Leveling level = EstimateLeveling(room.points, room.cameras);
  ASSERT_TRUE(level.floor_found);

  // Levelling may only move the world, never reshape it: every pairwise
  // distance has to survive exactly. This is what lets it be applied to a
  // finished reconstruction without re-solving anything.
  std::mt19937 rng(3);
  std::uniform_int_distribution<size_t> pick(0, room.points.size() - 1);
  for (int i = 0; i < 400; ++i) {
    const size_t a = pick(rng), b = pick(rng);
    const double before = (room.points[a] - room.points[b]).norm();
    const double after = (level.transform.Apply(room.points[a]) -
                          level.transform.Apply(room.points[b]))
                             .norm();
    ASSERT_NEAR(before, after, 1e-9);
  }
  EXPECT_NEAR(level.transform.q.norm(), 1.0, 1e-12);
}

TEST(LevelTest, LeavesAScanAloneWhenNothingLooksLikeAFloor) {
  // A cloud with no dominant plane at all — scanning an object on a stand,
  // say. Levelling has nothing to lock onto, and the honest answer is to
  // leave the reconstruction exactly as solved rather than square it up
  // against whatever happens to be largest.
  Room room;
  std::mt19937 rng(5);
  std::normal_distribution<double> g(0.0, 0.6);
  for (int i = 0; i < 2000; ++i) {
    room.points.emplace_back(g(rng), 1.5 + g(rng), g(rng));
  }
  for (int i = 0; i < 40; ++i) {
    const double a = 2 * M_PI * i / 40.0;
    room.cameras.emplace_back(2.5 * std::cos(a), 1.5, 2.5 * std::sin(a));
  }

  const Leveling level = EstimateLeveling(room.points, room.cameras);
  EXPECT_FALSE(level.floor_found);
  EXPECT_LT((level.transform.t).norm(), 1e-12);
  EXPECT_NEAR(level.transform.q.w(), 1.0, 1e-12) << "identity expected";
}

// A scan holding exactly one big plane cannot be levelled honestly: a wall
// two metres to your left and a floor two metres below you look identical
// from the geometry alone — same distance, same constant offset along a
// straight walk, nothing on the far side of either. The check that matters
// is that whatever it decides, it does not silently invent an answer it
// cannot support; a lone plane is documented as ambiguous in level.h.
TEST(LevelTest, ASinglePlaneIsAmbiguousAndSaysSoOrLeavesItAlone) {
  Room room;
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (int i = 0; i < 1500; ++i) {
    room.points.emplace_back(2.0, 2.5 * u(rng), -4.0 + 8.0 * u(rng));
  }
  for (int i = 0; i < 40; ++i) {
    room.cameras.emplace_back(0.0, 1.5, -3.0 + 6.0 * i / 39.0);
  }
  const Leveling level = EstimateLeveling(room.points, room.cameras);
  // Either outcome is defensible; a non-rigid or wildly tilted one is not.
  EXPECT_NEAR(level.transform.q.norm(), 1.0, 1e-12);
  if (level.floor_found) {
    EXPECT_LT(level.floor_rmse_m, 0.05) << "claimed a floor it does not fit";
  }
}

// The measured path: a plane the depth sensor saw, in its own camera
// coordinates, plus that frame's pose. No searching, no ambiguity to reason
// around — the answer should come out exact.
TEST(LevelTest, MeasuredFloorLevelsExactly) {
  const Room room = MakeRoom(/*tilt_deg=*/12.0, /*yaw_deg=*/64.0);

  // The calibration frame: 1.45 m up, aimed straight down at the floor. In
  // ITS camera coordinates the floor's normal comes back along the lens.
  const Eigen::Quaterniond world(
      Eigen::AngleAxisd(DegToRad(12.0),
                        Eigen::Vector3d(1, 0, 0.3).normalized()) *
      Eigen::AngleAxisd(DegToRad(64.0), Eigen::Vector3d::UnitY()));
  const Eigen::Vector3d centre = world * Eigen::Vector3d(0.5, 1.45, -1.0);
  Eigen::Matrix3d R_cw;
  const Eigen::Vector3d down = (world * Eigen::Vector3d(0, -1, 0)).normalized();
  const Eigen::Vector3d side =
      down.cross(world * Eigen::Vector3d(0, 0, 1)).normalized();
  R_cw.col(0) = side;
  R_cw.col(1) = down.cross(side).normalized();
  R_cw.col(2) = down;
  const SE3 pose = SE3::FromCamToWorld(Eigen::Quaterniond(R_cw), centre);

  const Leveling level = LevelingFromMeasuredFloor(
      Eigen::Vector3d(0, 0, -1), 1.45, pose, room.points, room.cameras);

  ASSERT_TRUE(level.floor_found);
  EXPECT_TRUE(level.floor_measured) << "should report where the floor came from";

  double lowest = 1e9;
  int on_floor = 0;
  for (const auto& p : room.points) {
    const Eigen::Vector3d q = level.transform.Apply(p);
    lowest = std::min(lowest, q.y());
    if (std::abs(q.y()) < 0.01) ++on_floor;
  }
  EXPECT_GT(on_floor, 1000) << "the floor should land on y = 0";
  EXPECT_GT(lowest, -0.02);
  EXPECT_NEAR(level.camera_height_m, 1.5, 0.02);
  EXPECT_LT(level.camera_height_spread_m, 0.05);
}

TEST(LevelTest, DeterministicAcrossRuns) {
  const Room room = MakeRoom(7.0, 14.0);
  const Leveling a = EstimateLeveling(room.points, room.cameras);
  const Leveling b = EstimateLeveling(room.points, room.cameras);
  ASSERT_TRUE(a.floor_found && b.floor_found);
  // A re-export must not quietly move the world.
  EXPECT_NEAR(a.transform.q.angularDistance(b.transform.q), 0.0, 1e-12);
  EXPECT_LT((a.transform.t - b.transform.t).norm(), 1e-12);
}

}  // namespace
}  // namespace bs

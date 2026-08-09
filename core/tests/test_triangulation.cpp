#include <gtest/gtest.h>

#include <random>

#include "geometry/triangulation.h"

namespace bs {
namespace {

const Intrinsics kK{600.0, 600.0, 479.5, 359.5, 960, 720};

SE3 CameraAt(double x, double yaw_deg = 0) {
  SE3 pose;
  pose.q = Eigen::Quaterniond(
      Eigen::AngleAxisd(DegToRad(yaw_deg), Eigen::Vector3d::UnitY()));
  pose.t = -(pose.q * Eigen::Vector3d(x, 0, 0));
  return pose;
}

TEST(Triangulation, ExactRecoveryTwoViews) {
  const SE3 pose_a = CameraAt(0.0);
  const SE3 pose_b = CameraAt(0.5);
  const Eigen::Vector3d X(0.3, -0.2, 4.0);

  const Eigen::Vector2d pa = kK.Project(pose_a.Apply(X));
  const Eigen::Vector2d pb = kK.Project(pose_b.Apply(X));

  const auto result = TriangulateTwoView(
      pose_a, pose_b, kK,
      {static_cast<float>(pa.x()), static_cast<float>(pa.y())},
      {static_cast<float>(pb.x()), static_cast<float>(pb.y())});
  ASSERT_TRUE(result.has_value());
  EXPECT_LT((result->point - X).norm(), 1e-6);
  EXPECT_LT(result->max_reproj_error_px, 1e-6);
  EXPECT_TRUE(result->InFrontOfAll());
}

TEST(Triangulation, MultiViewBeatsTwoViewUnderNoise) {
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 0.5);
  const Eigen::Vector3d X(-0.4, 0.3, 5.0);

  std::vector<Observation2D> all;
  for (int i = 0; i < 6; ++i) {
    const SE3 pose = CameraAt(-0.75 + 0.3 * i, 2.0 * i - 5.0);
    const Eigen::Vector2d p = kK.Project(pose.Apply(X));
    all.push_back({pose, kK,
                   {static_cast<float>(p.x() + noise(rng)),
                    static_cast<float>(p.y() + noise(rng))}});
  }

  const auto two = TriangulatePoint({all[0], all[1]});
  const auto six = TriangulatePoint(all);
  ASSERT_TRUE(two && six);
  EXPECT_LT((six->point - X).norm(), (two->point - X).norm() + 1e-3);
  EXPECT_LT((six->point - X).norm(), 0.02);
}

TEST(Triangulation, AngleMatchesClosedForm) {
  // Symmetric cameras at +-b/2 looking at a point at depth d on the axis:
  // angle = 2 atan((b/2) / d).
  const double b = 0.8, d = 4.0;
  const double expected = RadToDeg(2.0 * std::atan((b / 2.0) / d));
  const double angle = TriangulationAngleDeg(
      {-b / 2.0, 0, 0}, {b / 2.0, 0, 0}, {0, 0, d});
  EXPECT_NEAR(angle, expected, 1e-9);
}

TEST(Triangulation, DetectsBehindCamera) {
  const SE3 pose_a = CameraAt(0.0);
  const SE3 pose_b = CameraAt(0.5);
  // Feed inconsistent observations that triangulate behind the cameras:
  // both cameras look down +Z, so a point with negative depth shows up in
  // behind_camera_count rather than silently passing.
  const Eigen::Vector3d X(0.1, 0.1, -3.0);
  const Eigen::Vector3d xa = pose_a.Apply(X);
  const Eigen::Vector3d xb = pose_b.Apply(X);
  // Project manually despite negative z (mimics a mismatched correspondence).
  const Eigen::Vector2d pa(kK.fx * xa.x() / xa.z() + kK.cx,
                           kK.fy * xa.y() / xa.z() + kK.cy);
  const Eigen::Vector2d pb(kK.fx * xb.x() / xb.z() + kK.cx,
                           kK.fy * xb.y() / xb.z() + kK.cy);
  const auto result = TriangulateTwoView(
      pose_a, pose_b, kK,
      {static_cast<float>(pa.x()), static_cast<float>(pa.y())},
      {static_cast<float>(pb.x()), static_cast<float>(pb.y())});
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->InFrontOfAll());
}

TEST(Triangulation, RejectsDegenerateInput) {
  EXPECT_FALSE(TriangulatePoint({}).has_value());
  const SE3 pose = CameraAt(0.0);
  EXPECT_FALSE(TriangulatePoint({{pose, kK, {100, 100}}}).has_value());
}

}  // namespace
}  // namespace bs

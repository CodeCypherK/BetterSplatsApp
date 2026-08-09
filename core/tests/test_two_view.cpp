// Relative-pose estimation against synthetic ground truth.

#include <gtest/gtest.h>

#include <random>

#include "common/geometry.h"
#include "geometry/two_view.h"

namespace bs {
namespace {

struct SyntheticPair {
  std::vector<cv::Point2f> points_a;
  std::vector<cv::Point2f> points_b;
  SE3 rel_pose;  // ground truth: x_b = R x_a + t (unit t)
  Intrinsics K;
};

// Random scene points in front of camera A; camera B displaced by
// `baseline` and rotated by `rot_deg` around Y. Projections carry
// `noise_px` gaussian noise.
SyntheticPair MakePair(int n_points, double baseline, double rot_deg,
                       double noise_px, uint32_t seed,
                       bool planar_scene = false) {
  SyntheticPair out;
  out.K = {600.0, 600.0, 479.5, 359.5, 960, 720};

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> ux(-2.0, 2.0);
  std::uniform_real_distribution<double> uy(-1.5, 1.5);
  std::uniform_real_distribution<double> uz(2.0, 6.0);
  std::normal_distribution<double> noise(0.0, noise_px);

  const SE3 pose_a = SE3::Identity();
  SE3 pose_b;  // world(=camA frame) -> camB
  pose_b.q = Eigen::Quaterniond(
      Eigen::AngleAxisd(DegToRad(rot_deg), Eigen::Vector3d::UnitY()));
  const Eigen::Vector3d center_b(baseline, 0, 0);
  pose_b.t = -(pose_b.q * center_b);

  out.rel_pose = pose_b;  // A frame == world
  out.rel_pose.t.normalize();

  int made = 0;
  while (made < n_points) {
    const double z = planar_scene ? 4.0 : uz(rng);
    const Eigen::Vector3d X(ux(rng), uy(rng), z);
    const Eigen::Vector3d xa = pose_a.Apply(X);
    const Eigen::Vector3d xb = pose_b.Apply(X);
    if (xa.z() <= 0.1 || xb.z() <= 0.1) continue;
    const Eigen::Vector2d pa = out.K.Project(xa);
    const Eigen::Vector2d pb = out.K.Project(xb);
    if (pa.x() < 0 || pa.x() >= out.K.width || pa.y() < 0 ||
        pa.y() >= out.K.height || pb.x() < 0 || pb.x() >= out.K.width ||
        pb.y() < 0 || pb.y() >= out.K.height) {
      continue;
    }
    out.points_a.emplace_back(static_cast<float>(pa.x() + noise(rng)),
                              static_cast<float>(pa.y() + noise(rng)));
    out.points_b.emplace_back(static_cast<float>(pb.x() + noise(rng)),
                              static_cast<float>(pb.y() + noise(rng)));
    ++made;
  }
  return out;
}

double RotationErrorDeg(const SE3& a, const SE3& b) {
  return RadToDeg(AngularDistance(a.q, b.q));
}

double TranslationDirErrorDeg(const SE3& a, const SE3& b) {
  const double c =
      std::clamp(a.t.normalized().dot(b.t.normalized()), -1.0, 1.0);
  return RadToDeg(std::acos(c));
}

TEST(TwoView, RecoversPoseNoiseFree) {
  const SyntheticPair pair = MakePair(300, 0.4, 12.0, 0.0, 11);
  const TwoViewResult result =
      EstimateRelativePose(pair.points_a, pair.points_b, pair.K, {});
  ASSERT_TRUE(result.ok()) << static_cast<int>(result.failure);
  EXPECT_LT(RotationErrorDeg(result.rel_pose, pair.rel_pose), 0.05);
  EXPECT_LT(TranslationDirErrorDeg(result.rel_pose, pair.rel_pose), 0.1);
  EXPECT_GT(result.inliers_e, 280);
}

TEST(TwoView, RecoversPoseWithPixelNoise) {
  const SyntheticPair pair = MakePair(400, 0.35, 8.0, 0.4, 23);
  const TwoViewResult result =
      EstimateRelativePose(pair.points_a, pair.points_b, pair.K, {});
  ASSERT_TRUE(result.ok()) << static_cast<int>(result.failure);
  EXPECT_LT(RotationErrorDeg(result.rel_pose, pair.rel_pose), 0.1);
  EXPECT_LT(TranslationDirErrorDeg(result.rel_pose, pair.rel_pose), 0.5);
}

TEST(TwoView, RecoversPoseWithOutliers) {
  SyntheticPair pair = MakePair(300, 0.4, 10.0, 0.3, 31);
  // Corrupt 25% of correspondences.
  std::mt19937 rng(99);
  std::uniform_real_distribution<float> u(0.0f, 700.0f);
  for (size_t i = 0; i < pair.points_b.size(); i += 4) {
    pair.points_b[i] = {u(rng), u(rng)};
  }
  const TwoViewResult result =
      EstimateRelativePose(pair.points_a, pair.points_b, pair.K, {});
  ASSERT_TRUE(result.ok()) << static_cast<int>(result.failure);
  EXPECT_LT(RotationErrorDeg(result.rel_pose, pair.rel_pose), 0.15);
  EXPECT_LT(TranslationDirErrorDeg(result.rel_pose, pair.rel_pose), 1.0);
}

TEST(TwoView, RejectsPureRotation) {
  // Zero baseline: no parallax exists; the estimator must reject the pair
  // rather than produce a garbage translation. Depending on where the noise
  // sends the spurious solution this is caught by the parallax gate
  // (kRotationDominant) or by triangulations collapsing behind the cameras
  // (kCheiralityFailed) — both are correct rejections.
  const SyntheticPair pair = MakePair(300, 0.0, 10.0, 0.2, 47);
  const TwoViewResult result =
      EstimateRelativePose(pair.points_a, pair.points_b, pair.K, {});
  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(result.failure == TwoViewFailure::kRotationDominant ||
              result.failure == TwoViewFailure::kCheiralityFailed)
      << static_cast<int>(result.failure);
}

TEST(TwoView, RejectsTooFewMatches) {
  SyntheticPair pair = MakePair(6, 0.4, 10.0, 0.0, 5);
  const TwoViewResult result =
      EstimateRelativePose(pair.points_a, pair.points_b, pair.K, {});
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.failure, TwoViewFailure::kTooFewMatches);
}

}  // namespace
}  // namespace bs

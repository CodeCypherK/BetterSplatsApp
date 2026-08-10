// Aligning a recovered reconstruction component back into the main model.
// This is the step that lets the final solve repair a live tracking failure,
// so its failure modes matter: a wrong transform silently relocates a whole
// room, and a free scale silently resizes one.

#include <gtest/gtest.h>

#include <random>
#include <vector>

#include "final/component_align.h"

namespace bs {
namespace {

// A reproducible cloud with real spatial extent in all three axes.
std::vector<Eigen::Vector3d> MakeCloud(int n, uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(-2.5, 2.5);
  std::vector<Eigen::Vector3d> points;
  points.reserve(n);
  for (int i = 0; i < n; ++i) points.emplace_back(u(rng), u(rng), u(rng));
  return points;
}

Similarity MakeTransform(double scale, double yaw, double pitch,
                         const Eigen::Vector3d& t) {
  Similarity s;
  s.scale = scale;
  s.R = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
  s.t = t;
  return s;
}

TEST(ComponentAlign, RecoversKnownSimilarity) {
  const auto src = MakeCloud(40, 1);
  const Similarity truth =
      MakeTransform(1.35, 0.6, -0.25, {1.5, -0.75, 3.0});
  std::vector<Eigen::Vector3d> dst;
  for (const auto& p : src) dst.push_back(truth.Apply(p));

  Similarity fit;
  ASSERT_TRUE(SimilarityFromCorrespondences(src, dst, fit));
  EXPECT_NEAR(fit.scale, truth.scale, 1e-9);
  EXPECT_LT((fit.R - truth.R).norm(), 1e-9);
  EXPECT_LT((fit.t - truth.t).norm(), 1e-9);
  EXPECT_NEAR(fit.R.determinant(), 1.0, 1e-9);  // proper rotation, no mirror
}

TEST(ComponentAlign, RigidFitIgnoresScaleDifference) {
  const auto src = MakeCloud(40, 2);
  const Similarity truth = MakeTransform(1.2, 0.4, 0.1, {0.5, 0.25, -1.0});
  std::vector<Eigen::Vector3d> dst;
  for (const auto& p : src) dst.push_back(truth.Apply(p));

  // Both gauges metric -> the fit must NOT absorb a scale, even when the
  // data suggests one. (Letting it caused a 9.6% global scale error.)
  Similarity fit;
  ASSERT_TRUE(SimilarityFromCorrespondences(src, dst, fit,
                                            /*allow_scale=*/false));
  EXPECT_DOUBLE_EQ(fit.scale, 1.0);
  EXPECT_LT((fit.R - truth.R).norm(), 1e-9);  // rotation still recovered
}

TEST(ComponentAlign, RobustFitSurvivesMajorityOutliers) {
  const auto src = MakeCloud(120, 3);
  const Similarity truth = MakeTransform(1.0, -0.8, 0.2, {2.0, 0.5, -1.5});
  std::vector<Eigen::Vector3d> dst;
  for (const auto& p : src) dst.push_back(truth.Apply(p));

  // Only ~15% of correspondences are real — the measured situation when two
  // rooms share a doorway's worth of structure.
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> junk(-6.0, 6.0);
  for (size_t i = 18; i < dst.size(); ++i) {
    dst[i] = Eigen::Vector3d(junk(rng), junk(rng), junk(rng));
  }

  Similarity fit;
  int inliers = 0;
  ASSERT_TRUE(RobustSimilarity(src, dst, /*inlier_radius_m=*/0.05,
                               /*min_inliers=*/12, /*allow_scale=*/false, fit,
                               inliers));
  EXPECT_GE(inliers, 18);
  EXPECT_LT((fit.R - truth.R).norm(), 1e-6);
  EXPECT_LT((fit.t - truth.t).norm(), 1e-6);
}

TEST(ComponentAlign, RobustFitRefusesWhenThereIsNoOverlap) {
  const auto src = MakeCloud(60, 4);
  const auto dst = MakeCloud(60, 5);  // unrelated cloud: no true transform
  Similarity fit;
  int inliers = 0;
  EXPECT_FALSE(RobustSimilarity(src, dst, 0.05, 25, false, fit, inliers));
}

TEST(ComponentAlign, PoseTransformKeepsTheCameraOnTheSameStructure) {
  // A camera and a point it observes, both in the component's gauge. After
  // moving both into the target gauge the point must land in exactly the
  // same place in the camera's own frame — that invariance is what makes
  // the merged poses consistent with the merged structure.
  SE3 pose_component;
  pose_component.q =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.2, 1, 0.3)
                                                    .normalized()));
  pose_component.t = Eigen::Vector3d(0.4, -0.2, 1.1);

  const Similarity sim = MakeTransform(1.0, 0.9, -0.3, {3.0, 1.0, -2.0});
  const SE3 pose_target = TransformPose(pose_component, sim);

  for (const auto& X : MakeCloud(10, 6)) {
    const Eigen::Vector3d in_camera_before = pose_component.Apply(X);
    const Eigen::Vector3d in_camera_after = pose_target.Apply(sim.Apply(X));
    EXPECT_LT((in_camera_before - in_camera_after).norm(), 1e-9);
  }
  EXPECT_NEAR(pose_target.q.norm(), 1.0, 1e-9);
}

TEST(ComponentAlign, PoseTransformScalesTheBaselineUnderSimilarity) {
  // With a scaled gauge the camera centre must move with the world, so the
  // distance between two cameras scales by exactly the similarity's scale.
  SE3 a, b;
  a.q = Eigen::Quaterniond::Identity();
  a.t = Eigen::Vector3d::Zero();
  b.q = Eigen::Quaterniond::Identity();
  b.t = Eigen::Vector3d(-1.0, 0, 0);  // camera centre at x = +1

  const Similarity sim = MakeTransform(2.5, 0.3, 0.0, {1.0, 2.0, 3.0});
  const double before = (a.CameraCenter() - b.CameraCenter()).norm();
  const double after = (TransformPose(a, sim).CameraCenter() -
                        TransformPose(b, sim).CameraCenter())
                           .norm();
  EXPECT_NEAR(after, sim.scale * before, 1e-9);
}

}  // namespace
}  // namespace bs

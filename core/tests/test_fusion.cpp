// Fusion layer: LiDAR associations, adaptive texture weighting, scale
// estimation, residual Jacobians, and metric-scale recovery through BA.

#include <gtest/gtest.h>

#include <random>

#include "common/config.h"
#include "fusion/residuals.h"
#include "fusion/scale.h"
#include "io/float16.h"

namespace bs {
namespace {

const Intrinsics kDepthK{242.0, 242.0, 159.5, 119.5, 320, 240};
const Intrinsics kImageK{600.0, 600.0, 479.5, 359.5, 960, 720};

DepthImage PlaneDepthForPose(const SE3& pose, const Eigen::Vector3d& n_world,
                             double dist_world, const Intrinsics& K) {
  // World plane {X : n.X = dist} rendered from `pose` (world-to-camera).
  DepthImage depth;
  depth.width = K.width;
  depth.height = K.height;
  depth.f16.resize(static_cast<size_t>(K.width) * K.height);
  const SE3 cam_to_world = pose.Inverse();
  const Eigen::Vector3d center = pose.CameraCenter();
  const Eigen::Matrix3d R_cw = cam_to_world.q.toRotationMatrix();
  for (int y = 0; y < K.height; ++y) {
    for (int x = 0; x < K.width; ++x) {
      const Eigen::Vector3d dir = R_cw * K.Unproject(x, y, 1.0);
      const double denom = n_world.dot(dir);
      double z = NAN;
      if (std::abs(denom) > 1e-9) {
        const double t = (dist_world - n_world.dot(center)) / denom;
        if (t > 0) z = t;  // dir has unit camera-z, so t is z-depth
      }
      depth.f16[static_cast<size_t>(y) * K.width + x] =
          F32ToF16(static_cast<float>(z));
    }
  }
  return depth;
}

TEST(Fusion, AssociationGating) {
  const SE3 identity = SE3::Identity();
  const DepthFrame frame(
      PlaneDepthForPose(identity, {0, 0, 1}, 3.0, kDepthK), kDepthK);

  // Point on the plane: association succeeds with strong weight.
  const auto good = MakeLidarAssociation(frame, identity, {0.2, -0.1, 3.0},
                                         /*w_tex=*/1.0, /*gate_sigmas=*/3.0);
  ASSERT_TRUE(good.has_value());
  EXPECT_GT(good->sqrt_weight, 0.9);
  EXPECT_NEAR(good->sigma_m, 0.01 + 0.008 * 9.0, 2e-3);

  // Floater 40 cm in front of the wall: rejected by the gate.
  EXPECT_FALSE(MakeLidarAssociation(frame, identity, {0.2, -0.1, 2.6}, 1.0, 3.0)
                   .has_value());

  // Behind the camera / outside the image: rejected.
  EXPECT_FALSE(MakeLidarAssociation(frame, identity, {0, 0, -1.0}, 1.0, 3.0)
                   .has_value());
  EXPECT_FALSE(MakeLidarAssociation(frame, identity, {50.0, 0, 3.0}, 1.0, 3.0)
                   .has_value());

  // w_tex scales the weight continuously.
  const auto weak = MakeLidarAssociation(frame, identity, {0.2, -0.1, 3.0},
                                         /*w_tex=*/0.15, 3.0);
  ASSERT_TRUE(weak.has_value());
  EXPECT_NEAR(weak->sqrt_weight, good->sqrt_weight * std::sqrt(0.15), 1e-6);
}

TEST(Fusion, TextureWeightAdaptivity) {
  // Blank wall, no parallax: full LiDAR anchoring.
  EXPECT_DOUBLE_EQ(TextureWeight(0.0, 0.0), 1.0);
  // Strong texture AND strong triangulation: floor only.
  EXPECT_DOUBLE_EQ(TextureWeight(30.0, 6.0), 0.15);
  // Strong texture but weak parallax: still strongly anchored.
  EXPECT_GT(TextureWeight(30.0, 0.5), 0.85);
  // Continuous in between.
  const double mid = TextureWeight(12.5, 2.0);
  EXPECT_GT(mid, 0.5);
  EXPECT_LT(mid, 1.0);
}

TEST(Fusion, ScaleEstimation) {
  std::mt19937 rng(5);
  std::normal_distribution<double> tight(1.4, 0.02);

  std::vector<double> few;
  for (int i = 0; i < 10; ++i) few.push_back(tight(rng));
  EXPECT_FALSE(EstimateScale(few).locked);  // too few samples

  std::vector<double> good;
  for (int i = 0; i < 100; ++i) good.push_back(tight(rng));
  const ScaleEstimate locked = EstimateScale(good);
  EXPECT_TRUE(locked.locked);
  EXPECT_NEAR(locked.scale, 1.4, 0.02);

  // 20% gross outliers: median still lands on the mode, still locks.
  std::vector<double> contaminated = good;
  for (int i = 0; i < 25; ++i) contaminated.push_back(8.0);
  const ScaleEstimate robust = EstimateScale(contaminated);
  EXPECT_NEAR(robust.scale, 1.4, 0.05);

  // Wide multi-modal spread: refuses to lock.
  std::uniform_real_distribution<double> wide(0.5, 3.0);
  std::vector<double> spread;
  for (int i = 0; i < 100; ++i) spread.push_back(wide(rng));
  EXPECT_FALSE(EstimateScale(spread).locked);

  // Garbage values are dropped.
  const ScaleEstimate cleaned =
      EstimateScale({NAN, -1.0, 0.0, 1.5, 1.5, 1.5});
  EXPECT_EQ(cleaned.samples, 3);
}

TEST(Fusion, LidarResidualJacobianMatchesNumericDiff) {
  const SE3 pose = SE3::Identity();
  const DepthFrame frame(
      PlaneDepthForPose(pose, Eigen::Vector3d(0.15, -0.05, 1.0).normalized(),
                        2.8, kDepthK),
      kDepthK);
  const DepthLookup lookup(frame);
  LidarAssociation assoc;
  assoc.sqrt_weight = 0.8;
  assoc.sigma_m = 0.03;

  std::unique_ptr<ceres::CostFunction> autodiff(
      LidarDepthResidual::Create(&lookup, assoc));
  std::unique_ptr<ceres::CostFunction> numeric(
      new ceres::NumericDiffCostFunction<LidarDepthResidual, ceres::CENTRAL, 1,
                                         4, 3, 3>(
          new LidarDepthResidual(&lookup, assoc)));

  const double q[4] = {0.02, -0.01, 0.015, 0.9995};
  const double t[3] = {0.05, -0.03, 0.1};
  const double X[3] = {0.3, -0.2, 2.6};
  const double* params[3] = {q, t, X};

  double r_a = 0, r_n = 0;
  double ja_q[4], ja_t[3], ja_x[3];
  double jn_q[4], jn_t[3], jn_x[3];
  double* jac_a[3] = {ja_q, ja_t, ja_x};
  double* jac_n[3] = {jn_q, jn_t, jn_x};

  ASSERT_TRUE(autodiff->Evaluate(params, &r_a, jac_a));
  ASSERT_TRUE(numeric->Evaluate(params, &r_n, jac_n));
  ASSERT_NE(r_a, 0.0);  // the association is live, not gated out
  EXPECT_NEAR(r_a, r_n, 1e-9);
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(ja_q[i], jn_q[i], 1e-4 + 1e-3 * std::abs(ja_q[i])) << "q" << i;
  }
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(ja_t[i], jn_t[i], 1e-4 + 1e-3 * std::abs(ja_t[i])) << "t" << i;
    EXPECT_NEAR(ja_x[i], jn_x[i], 1e-4 + 1e-3 * std::abs(ja_x[i])) << "X" << i;
  }
}

// The load-bearing M3 test: a two-camera scene initialized at the wrong
// global scale must be pulled back to metric by LiDAR depth residuals while
// reprojection residuals keep the image geometry consistent.
TEST(Fusion, BundleAdjustmentRecoversMetricScale) {
  // Ground truth: cam0 at origin; cam1 0.4 m to the right; wall plane at
  // z = 3 (world normal +Z, both cameras face it).
  const SE3 pose0 = SE3::Identity();
  SE3 pose1;
  pose1.q = Eigen::Quaterniond(
      Eigen::AngleAxisd(DegToRad(4.0), Eigen::Vector3d::UnitY()));
  pose1.t = -(pose1.q * Eigen::Vector3d(0.4, 0, 0));

  const Eigen::Vector3d plane_n(0, 0, 1);
  const double plane_d = 3.0;

  // Points on the wall visible in both views.
  std::vector<Eigen::Vector3d> points_gt;
  for (double x = -1.2; x <= 1.6; x += 0.2) {
    for (double y = -0.9; y <= 0.9; y += 0.2) {
      points_gt.emplace_back(x, y, plane_d);
    }
  }

  // Depth frames rendered from ground truth (the immutable RAW layer).
  const DepthFrame frame0(
      PlaneDepthForPose(pose0, plane_n, plane_d, kDepthK), kDepthK);
  const DepthFrame frame1(
      PlaneDepthForPose(pose1, plane_n, plane_d, kDepthK), kDepthK);
  const DepthLookup lookup0(frame0);
  const DepthLookup lookup1(frame1);

  // Observations with mild pixel noise.
  std::mt19937 rng(11);
  std::normal_distribution<double> px_noise(0.0, 0.3);
  struct Obs {
    int cam;
    size_t point;
    double u, v;
  };
  std::vector<Obs> observations;
  for (size_t i = 0; i < points_gt.size(); ++i) {
    for (int cam = 0; cam < 2; ++cam) {
      const SE3& pose = cam == 0 ? pose0 : pose1;
      const Eigen::Vector3d xc = pose.Apply(points_gt[i]);
      if (xc.z() <= 0.1) continue;
      const Eigen::Vector2d px = kImageK.Project(xc);
      if (px.x() < 0 || px.x() >= kImageK.width || px.y() < 0 ||
          px.y() >= kImageK.height) {
        continue;
      }
      observations.push_back(
          {cam, i, px.x() + px_noise(rng), px.y() + px_noise(rng)});
    }
  }

  // Initialize at 1.25x scale: SfM gauge error to be corrected by LiDAR.
  const double bad_scale = 1.25;
  double q0[4], t0[3], q1[4], t1[3];
  PoseToBlocks(pose0, q0, t0);
  SE3 pose1_scaled = pose1;
  pose1_scaled.t *= bad_scale;
  PoseToBlocks(pose1_scaled, q1, t1);

  std::vector<std::array<double, 3>> points(points_gt.size());
  for (size_t i = 0; i < points_gt.size(); ++i) {
    const Eigen::Vector3d scaled = points_gt[i] * bad_scale;
    points[i] = {scaled.x(), scaled.y(), scaled.z()};
  }

  ceres::Problem problem;
  for (const auto& obs : observations) {
    double* q = obs.cam == 0 ? q0 : q1;
    double* t = obs.cam == 0 ? t0 : t1;
    problem.AddResidualBlock(
        ReprojectionResidual::Create(obs.u, obs.v, kImageK),
        new ceres::HuberLoss(2.0), q, t, points[obs.point].data());
  }

  // LiDAR associations against CURRENT (scaled, wrong) geometry — the gate
  // must operate on the initialization the solver actually sees. Wall
  // points are "blank wall" evidence: w_tex = 1.
  int lidar_added = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    const Eigen::Vector3d X(points[i][0], points[i][1], points[i][2]);
    for (int cam = 0; cam < 2; ++cam) {
      const SE3 pose_init =
          cam == 0 ? pose0 : PoseFromBlocks(q1, t1);
      const DepthFrame& frame = cam == 0 ? frame0 : frame1;
      const DepthLookup& lookup = cam == 0 ? lookup0 : lookup1;
      // Wide gate: the 25% scale error is exactly what we must associate
      // through (real code widens the gate before the gauge locks).
      const auto assoc =
          MakeLidarAssociation(frame, pose_init, X, 1.0, /*gate_sigmas=*/30.0);
      if (!assoc) continue;
      double* q = cam == 0 ? q0 : q1;
      double* t = cam == 0 ? t0 : t1;
      problem.AddResidualBlock(LidarDepthResidual::Create(&lookup, *assoc),
                               new ceres::CauchyLoss(1.0), q, t,
                               points[i].data());
      ++lidar_added;
    }
  }
  ASSERT_GT(lidar_added, 100);

  problem.SetManifold(q0, new ceres::EigenQuaternionManifold);
  problem.SetManifold(q1, new ceres::EigenQuaternionManifold);
  problem.SetParameterBlockConstant(q0);
  problem.SetParameterBlockConstant(t0);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = 60;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  const SE3 solved1 = PoseFromBlocks(q1, t1);
  const double baseline = (solved1.CameraCenter() - pose0.CameraCenter()).norm();
  EXPECT_NEAR(baseline, 0.4, 0.004) << summary.BriefReport();  // < 1%

  // Points return to the metric wall.
  double depth_err = 0;
  for (const auto& p : points) depth_err += std::abs(p[2] - plane_d);
  depth_err /= points.size();
  EXPECT_LT(depth_err, 0.02);

  // Rotation must not be disturbed (bound reflects the 0.3 px observation
  // noise on a single-plane scene, not a LiDAR-induced bias).
  EXPECT_LT(RadToDeg(AngularDistance(solved1.q, pose1.q)), 0.2);
}


// A large PnP inlier count is not proof of a correct pose: in a room of
// repeated texture RANSAC can find a big consistent set somewhere wrong.
// Accepting one such pose is unrecoverable, because the local map then
// projects outside the predicted frustum and no later frame can re-acquire
// by tracking. Measured on a walking sweep: a 5.9 m single-frame jump was
// accepted with 92/114 inliers, and tracking never came back.
TEST(Tracking, MotionPlausibilityRejectsImpossibleJumps) {
  const double dt = 1.0 / 30.0;
  const double max_speed = 4.0;   // m/s
  const double max_rot = 300.0;   // deg/s

  SE3 previous;
  previous.q = Eigen::Quaterniond::Identity();
  previous.t = Eigen::Vector3d::Zero();

  auto at_center = [](const Eigen::Vector3d& c,
                      const Eigen::Quaterniond& q = Eigen::Quaterniond::Identity()) {
    return SE3::FromCamToWorld(q, c);
  };

  // Ordinary handheld motion at 30 fps: ~3 cm and a degree or two.
  EXPECT_TRUE(MotionIsPlausible(previous, at_center({0.03, 0.0, 0.0}), dt,
                                max_speed, max_rot));
  // The measured failure: 5.9 m in a single frame.
  EXPECT_FALSE(MotionIsPlausible(previous, at_center({5.9, 0.0, 0.0}), dt,
                                 max_speed, max_rot));
  // Just inside and just outside the translation limit.
  const double limit = max_speed * dt;
  EXPECT_TRUE(MotionIsPlausible(previous, at_center({limit * 0.95, 0, 0}), dt,
                                max_speed, max_rot));
  EXPECT_FALSE(MotionIsPlausible(previous, at_center({limit * 1.05, 0, 0}), dt,
                                 max_speed, max_rot));

  // Rotation is bounded independently of translation: a 120 deg flip in one
  // frame is impossible even with the camera centre unmoved.
  const Eigen::Quaterniond flipped(
      Eigen::AngleAxisd(DegToRad(120.0), Eigen::Vector3d::UnitY()));
  EXPECT_FALSE(MotionIsPlausible(previous, at_center({0, 0, 0}, flipped), dt,
                                 max_speed, max_rot));
  const Eigen::Quaterniond nudged(
      Eigen::AngleAxisd(DegToRad(3.0), Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(MotionIsPlausible(previous, at_center({0, 0, 0}, nudged), dt,
                                max_speed, max_rot));

  // A longer gap between frames proportionally allows more motion — the
  // limit is a speed, not a per-frame distance.
  EXPECT_TRUE(MotionIsPlausible(previous, at_center({0.5, 0, 0}), 0.25,
                                max_speed, max_rot));
  // With no usable time reference nothing can be judged, so nothing is
  // rejected (the caller has not tracked a previous frame yet).
  EXPECT_TRUE(MotionIsPlausible(previous, at_center({99.0, 0, 0}), 0.0,
                                max_speed, max_rot));
}

// The turn rate that drives SLOW DOWN sits far below the rejection bound:
// the pose is fine, the user is simply outrunning the mapper.
TEST(MotionTest, TurnRateSeparatesComfortableFromUnmappable) {
  const double dt = 1.0 / 30.0;
  const EngineConfig config;

  SE3 previous;
  previous.q = Eigen::Quaterniond::Identity();
  previous.t = Eigen::Vector3d::Zero();
  auto turned = [](double deg) {
    return SE3::FromCamToWorld(
        Eigen::Quaterniond(
            Eigen::AngleAxisd(DegToRad(deg), Eigen::Vector3d::UnitY())),
        Eigen::Vector3d::Zero());
  };

  // A degree per frame is a calm scanning pan: 30 deg/s, no warning.
  EXPECT_NEAR(TurnRateDps(previous, turned(1.0), dt), 30.0, 1e-6);
  EXPECT_LT(TurnRateDps(previous, turned(1.0), dt), config.track_warn_rot_dps);

  // The measured killer: 4 deg/frame is 120 deg/s. It must warn...
  const double fast = TurnRateDps(previous, turned(4.0), dt);
  EXPECT_NEAR(fast, 120.0, 1e-6);
  EXPECT_GT(fast, config.track_warn_rot_dps);
  // ...but it is a perfectly makeable motion, so it must NOT be rejected.
  // Warning and rejecting are different jobs; conflating them would throw
  // away good poses instead of telling the user to ease off.
  EXPECT_LT(fast, config.track_max_rot_dps);
  EXPECT_TRUE(MotionIsPlausible(previous, turned(4.0), dt,
                                config.track_max_speed_mps,
                                config.track_max_rot_dps));

  // It is a rate, not a per-frame angle: the same 4 deg over four times the
  // interval is a quarter of the rate, and comfortable.
  EXPECT_NEAR(TurnRateDps(previous, turned(4.0), 4.0 * dt), 30.0, 1e-6);
  EXPECT_LT(TurnRateDps(previous, turned(4.0), 4.0 * dt),
            config.track_warn_rot_dps);

  // No time reference: nothing to report rather than a divide by zero.
  EXPECT_EQ(TurnRateDps(previous, turned(90.0), 0.0), 0.0);
}

}  // namespace
}  // namespace bs

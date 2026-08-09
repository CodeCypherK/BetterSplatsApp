#include "geometry/two_view.h"

#include <algorithm>

#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>

#include "common/log.h"
#include "geometry/triangulation.h"

namespace bs {

namespace {

// Sampson epipolar error of one normalized correspondence under the
// essential matrix E = [t]x R, scaled by `focal` into pixel-ish units.
// Parameters: q as Eigen coeff order (x,y,z,w) on the quaternion manifold,
// t on the unit sphere (translation scale is unobservable).
struct SampsonResidual {
  SampsonResidual(const Eigen::Vector2d& xa, const Eigen::Vector2d& xb,
                  double focal)
      : xa_(xa), xb_(xb), focal_(focal) {}

  template <typename T>
  bool operator()(const T* const q_xyzw, const T* const t_raw,
                  T* residual) const {
    const Eigen::Quaternion<T> q(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]);
    const Eigen::Matrix<T, 3, 3> R = q.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> t_skew;
    t_skew << T(0), -t_raw[2], t_raw[1],
              t_raw[2], T(0), -t_raw[0],
              -t_raw[1], t_raw[0], T(0);
    const Eigen::Matrix<T, 3, 3> E = t_skew * R;

    const Eigen::Matrix<T, 3, 1> xa(T(xa_.x()), T(xa_.y()), T(1));
    const Eigen::Matrix<T, 3, 1> xb(T(xb_.x()), T(xb_.y()), T(1));

    const Eigen::Matrix<T, 3, 1> Exa = E * xa;
    const Eigen::Matrix<T, 3, 1> Etxb = E.transpose() * xb;
    const T num = xb.dot(Exa);
    const T denom_sq = Exa(0) * Exa(0) + Exa(1) * Exa(1) +
                       Etxb(0) * Etxb(0) + Etxb(1) * Etxb(1);
    residual[0] = T(focal_) * num / ceres::sqrt(denom_sq + T(1e-18));
    return true;
  }

  Eigen::Vector2d xa_, xb_;
  double focal_;
};

// Sampson error (pixels) of one correspondence under pose (world=A frame).
double SampsonErrorPx(const cv::Point2f& pa, const cv::Point2f& pb,
                      const Intrinsics& K, const SE3& pose) {
  const Eigen::Matrix3d R = pose.q.toRotationMatrix();
  Eigen::Matrix3d t_skew;
  t_skew << 0, -pose.t.z(), pose.t.y(),
            pose.t.z(), 0, -pose.t.x(),
            -pose.t.y(), pose.t.x(), 0;
  const Eigen::Matrix3d E = t_skew * R;
  const Eigen::Vector3d xa((pa.x - K.cx) / K.fx, (pa.y - K.cy) / K.fy, 1.0);
  const Eigen::Vector3d xb((pb.x - K.cx) / K.fx, (pb.y - K.cy) / K.fy, 1.0);
  const Eigen::Vector3d Exa = E * xa;
  const Eigen::Vector3d Etxb = E.transpose() * xb;
  const double num = xb.dot(Exa);
  const double denom_sq = Exa(0) * Exa(0) + Exa(1) * Exa(1) +
                          Etxb(0) * Etxb(0) + Etxb(1) * Etxb(1);
  return 0.5 * (K.fx + K.fy) * std::abs(num) / std::sqrt(denom_sq + 1e-18);
}

void RefineOnMask(const std::vector<cv::Point2f>& points_a,
                  const std::vector<cv::Point2f>& points_b,
                  const std::vector<uint8_t>& mask, const Intrinsics& K,
                  SE3& pose) {
  double q_param[4] = {pose.q.x(), pose.q.y(), pose.q.z(), pose.q.w()};
  double t_param[3] = {pose.t.x(), pose.t.y(), pose.t.z()};

  ceres::Problem problem;
  ceres::LossFunction* loss = new ceres::HuberLoss(1.0);
  const double focal = 0.5 * (K.fx + K.fy);
  int added = 0;
  for (size_t i = 0; i < points_a.size(); ++i) {
    if (i >= mask.size() || mask[i] == 0) continue;
    const Eigen::Vector2d xa((points_a[i].x - K.cx) / K.fx,
                             (points_a[i].y - K.cy) / K.fy);
    const Eigen::Vector2d xb((points_b[i].x - K.cx) / K.fx,
                             (points_b[i].y - K.cy) / K.fy);
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<SampsonResidual, 1, 4, 3>(
            new SampsonResidual(xa, xb, focal)),
        loss, q_param, t_param);
    ++added;
  }
  if (added < 8) return;

  problem.SetManifold(q_param, new ceres::EigenQuaternionManifold);
  problem.SetManifold(t_param, new ceres::SphereManifold<3>);

  ceres::Solver::Options options;
  options.max_num_iterations = 20;
  options.logging_type = ceres::SILENT;
  options.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  pose.q = Eigen::Quaterniond(q_param[3], q_param[0], q_param[1], q_param[2])
               .normalized();
  pose.t = Eigen::Vector3d(t_param[0], t_param[1], t_param[2]).normalized();
}

// Polishes (R, t) with iterated refine + inlier re-collection. recoverPose
// alone leaves ~1-2 deg of translation-direction error at phone noise
// levels, and refining only on the RANSAC inlier set inherits its selection
// bias (genuine correspondences dropped by the provisional model stay
// dropped). Re-classifying against the refined model de-biases — standard
// locally-optimized-RANSAC practice.
void RefineRelativePose(const std::vector<cv::Point2f>& points_a,
                        const std::vector<cv::Point2f>& points_b,
                        std::vector<uint8_t>& inlier_mask, const Intrinsics& K,
                        double thresh_px, SE3& pose) {
  RefineOnMask(points_a, points_b, inlier_mask, K, pose);
  for (int round = 0; round < 2; ++round) {
    std::vector<uint8_t> recollected(points_a.size(), 0);
    int count = 0;
    for (size_t i = 0; i < points_a.size(); ++i) {
      if (SampsonErrorPx(points_a[i], points_b[i], K, pose) < thresh_px) {
        recollected[i] = 1;
        ++count;
      }
    }
    if (count < 8) return;
    inlier_mask = std::move(recollected);
    RefineOnMask(points_a, points_b, inlier_mask, K, pose);
  }
}

}  // namespace

TwoViewResult EstimateRelativePose(const std::vector<cv::Point2f>& points_a,
                                   const std::vector<cv::Point2f>& points_b,
                                   const Intrinsics& K,
                                   const TwoViewOptions& options) {
  TwoViewResult result;
  if (points_a.size() != points_b.size() || points_a.size() < 8) {
    result.failure = TwoViewFailure::kTooFewMatches;
    return result;
  }

  cv::Mat camera = (cv::Mat_<double>(3, 3) << K.fx, 0, K.cx, 0, K.fy, K.cy,
                    0, 0, 1);

  cv::Mat e_mask;
  cv::Mat E = cv::findEssentialMat(points_a, points_b, camera, cv::USAC_MAGSAC,
                                   options.confidence, options.ransac_thresh_px,
                                   e_mask);
  if (E.empty() || E.rows != 3 || E.cols != 3) {
    result.failure = TwoViewFailure::kEstimationFailed;
    return result;
  }
  result.inliers_e = cv::countNonZero(e_mask);
  if (result.inliers_e < 8) {
    result.failure = TwoViewFailure::kTooFewMatches;
    return result;
  }

  if (options.estimate_homography) {
    cv::Mat h_mask;
    cv::Mat H = cv::findHomography(points_a, points_b, cv::USAC_MAGSAC,
                                   options.ransac_thresh_px, h_mask, 2000,
                                   options.confidence);
    result.inliers_h = H.empty() ? 0 : cv::countNonZero(h_mask);
    result.planar_ambiguous =
        result.inliers_h > 0.85 * static_cast<double>(result.inliers_e);
  }

  cv::Mat R, t;
  cv::Mat pose_mask = e_mask.clone();
  const int cheirality =
      cv::recoverPose(E, points_a, points_b, camera, R, t, pose_mask);
  result.cheirality_inliers = cheirality;
  if (cheirality <
      options.min_cheirality_frac * static_cast<double>(result.inliers_e)) {
    result.failure = TwoViewFailure::kCheiralityFailed;
    return result;
  }

  Eigen::Matrix3d rot;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) rot(r, c) = R.at<double>(r, c);
  }
  result.rel_pose.q = Eigen::Quaterniond(rot).normalized();
  result.rel_pose.t = Eigen::Vector3d(t.at<double>(0), t.at<double>(1),
                                      t.at<double>(2));
  const double norm = result.rel_pose.t.norm();
  if (norm > 1e-12) result.rel_pose.t /= norm;

  result.inlier_mask.assign(pose_mask.begin<uint8_t>(), pose_mask.end<uint8_t>());
  RefineRelativePose(points_a, points_b, result.inlier_mask, K,
                     2.0 * options.ransac_thresh_px, result.rel_pose);

  // Parallax gate: with no real baseline, E estimation still "succeeds" on
  // noise but every triangulation collapses to near-zero angle.
  std::vector<double> angles;
  const SE3 identity = SE3::Identity();
  for (size_t i = 0; i < points_a.size(); ++i) {
    if (i >= result.inlier_mask.size() || result.inlier_mask[i] == 0) continue;
    const auto tri = TriangulateTwoView(identity, result.rel_pose, K,
                                        points_a[i], points_b[i]);
    if (!tri || !tri->InFrontOfAll()) continue;
    angles.push_back(tri->max_angle_deg);
  }
  if (angles.size() < 8) {
    result.failure = TwoViewFailure::kCheiralityFailed;
    return result;
  }
  std::nth_element(angles.begin(), angles.begin() + angles.size() / 2,
                   angles.end());
  result.median_tri_angle_deg = angles[angles.size() / 2];
  if (result.median_tri_angle_deg < options.min_median_tri_angle_deg) {
    result.failure = TwoViewFailure::kRotationDominant;
    return result;
  }

  result.failure = TwoViewFailure::kNone;
  return result;
}

}  // namespace bs

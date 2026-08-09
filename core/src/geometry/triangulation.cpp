#include "geometry/triangulation.h"

#include <cmath>

namespace bs {

std::optional<TriangulationResult> TriangulatePoint(
    const std::vector<Observation2D>& observations) {
  const size_t n = observations.size();
  if (n < 2) return std::nullopt;

  // DLT: for each view, two rows of A x = 0 with x homogeneous.
  Eigen::MatrixXd A(2 * n, 4);
  for (size_t i = 0; i < n; ++i) {
    const auto& obs = observations[i];
    Eigen::Matrix<double, 3, 4> P;
    P.leftCols<3>() = obs.pose.q.toRotationMatrix();
    P.col(3) = obs.pose.t;
    Eigen::Matrix3d K_mat;
    K_mat << obs.K.fx, 0, obs.K.cx, 0, obs.K.fy, obs.K.cy, 0, 0, 1;
    P = K_mat * P;

    const double u = obs.px.x;
    const double v = obs.px.y;
    A.row(2 * i) = u * P.row(2) - P.row(0);
    A.row(2 * i + 1) = v * P.row(2) - P.row(1);
  }

  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
  const Eigen::Vector4d h = svd.matrixV().col(3);
  if (std::abs(h(3)) < 1e-12) return std::nullopt;

  TriangulationResult result;
  result.point = h.head<3>() / h(3);

  double err_sum = 0;
  for (size_t i = 0; i < n; ++i) {
    const auto& obs = observations[i];
    const Eigen::Vector3d x_cam = obs.pose.Apply(result.point);
    if (x_cam.z() <= 0) {
      ++result.behind_camera_count;
      continue;
    }
    const Eigen::Vector2d proj = obs.K.Project(x_cam);
    const double err = (proj - Eigen::Vector2d(obs.px.x, obs.px.y)).norm();
    err_sum += err;
    result.max_reproj_error_px = std::max(result.max_reproj_error_px, err);
  }
  const int in_front = static_cast<int>(n) - result.behind_camera_count;
  result.mean_reproj_error_px = in_front > 0 ? err_sum / in_front : 1e9;

  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      const double angle = TriangulationAngleDeg(
          observations[i].pose.CameraCenter(),
          observations[j].pose.CameraCenter(), result.point);
      result.max_angle_deg = std::max(result.max_angle_deg, angle);
    }
  }
  return result;
}

std::optional<TriangulationResult> TriangulateTwoView(
    const SE3& pose_a, const SE3& pose_b, const Intrinsics& K,
    const cv::Point2f& px_a, const cv::Point2f& px_b) {
  return TriangulatePoint({{pose_a, K, px_a}, {pose_b, K, px_b}});
}

double TriangulationAngleDeg(const Eigen::Vector3d& center_a,
                             const Eigen::Vector3d& center_b,
                             const Eigen::Vector3d& point) {
  const Eigen::Vector3d ray_a = (center_a - point).normalized();
  const Eigen::Vector3d ray_b = (center_b - point).normalized();
  const double cos_angle = std::clamp(ray_a.dot(ray_b), -1.0, 1.0);
  return RadToDeg(std::acos(cos_angle));
}

}  // namespace bs

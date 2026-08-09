// bs_selftest: tiny known-answer problems through Eigen, OpenCV and Ceres.
// Proves the full dependency chain compiles, links and computes correctly on
// the running platform (Linux CI, macOS CI cross-checks, and the physical
// iPhone via the app's diagnostics screen).

#include <cmath>
#include <cstdio>
#include <cstring>

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "bs/bs_api.h"

namespace {

bool SelftestEigen(char* line, size_t len) {
  Eigen::Matrix3d rot =
      Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const double det = rot.determinant();
  const double orth = (rot * rot.transpose() - Eigen::Matrix3d::Identity()).norm();
  const bool ok = std::abs(det - 1.0) < 1e-12 && orth < 1e-12;
  std::snprintf(line, len, "eigen %s (det=%.3f)", ok ? "ok" : "FAIL", det);
  return ok;
}

bool SelftestOpenCV(char* line, size_t len) {
  // Detect ORB features on a synthetic checkerboard: deterministic corners.
  cv::Mat img(240, 320, CV_8UC1, cv::Scalar(0));
  for (int y = 0; y < img.rows; ++y) {
    for (int x = 0; x < img.cols; ++x) {
      img.at<uint8_t>(y, x) = (((x / 20) + (y / 20)) % 2) ? 220 : 30;
    }
  }
  cv::GaussianBlur(img, img, cv::Size(3, 3), 0.8);
  auto orb = cv::ORB::create(200);
  std::vector<cv::KeyPoint> kps;
  cv::Mat desc;
  orb->detectAndCompute(img, cv::noArray(), kps, desc);
  const bool ok = kps.size() >= 50 && desc.rows == static_cast<int>(kps.size());
  std::snprintf(line, len, "opencv %s (%s, %zu kps)", ok ? "ok" : "FAIL",
                CV_VERSION, kps.size());
  return ok;
}

struct QuadResidual {
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    residual[0] = T(10.0) - x[0];
    return true;
  }
};

bool SelftestCeres(char* line, size_t len) {
  double x = 0.5;
  ceres::Problem problem;
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<QuadResidual, 1, 1>(new QuadResidual), nullptr,
      &x);
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  const bool ok = summary.termination_type == ceres::CONVERGENCE &&
                  std::abs(x - 10.0) < 1e-6;
  std::snprintf(line, len, "ceres %s (x=%.6f)", ok ? "ok" : "FAIL", x);
  return ok;
}

}  // namespace

extern "C" bs_result bs_selftest(char* buf, size_t buf_len) {
  if (buf == nullptr || buf_len == 0) return BS_ERR_INVALID_ARGUMENT;

  char eigen_line[96], cv_line[96], ceres_line[96];
  const bool eigen_ok = SelftestEigen(eigen_line, sizeof(eigen_line));
  const bool cv_ok = SelftestOpenCV(cv_line, sizeof(cv_line));
  const bool ceres_ok = SelftestCeres(ceres_line, sizeof(ceres_line));

  std::snprintf(buf, buf_len, "%s; %s; %s", eigen_line, cv_line, ceres_line);
  return (eigen_ok && cv_ok && ceres_ok) ? BS_OK : BS_ERR_INTERNAL;
}

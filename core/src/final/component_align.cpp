#include "final/component_align.h"

#include <algorithm>
#include <cmath>
#include <random>

#include <Eigen/SVD>

namespace bs {

bool SimilarityFromCorrespondences(const std::vector<Eigen::Vector3d>& src,
                                   const std::vector<Eigen::Vector3d>& dst,
                                   Similarity& out, bool allow_scale) {
  const size_t n = src.size();
  if (n < 3 || dst.size() != n) return false;

  Eigen::Vector3d mean_src = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_dst = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < n; ++i) {
    mean_src += src[i];
    mean_dst += dst[i];
  }
  mean_src /= static_cast<double>(n);
  mean_dst /= static_cast<double>(n);

  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  double var_src = 0;
  for (size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d a = src[i] - mean_src;
    const Eigen::Vector3d b = dst[i] - mean_dst;
    cov += b * a.transpose();
    var_src += a.squaredNorm();
  }
  cov /= static_cast<double>(n);
  var_src /= static_cast<double>(n);
  if (var_src < 1e-12) return false;

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
  // Reflection guard: force a proper rotation (det = +1).
  if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0) {
    S(2, 2) = -1;
  }
  out.R = svd.matrixU() * S * svd.matrixV().transpose();
  if (allow_scale) {
    out.scale = (svd.singularValues().asDiagonal() * S).trace() / var_src;
    if (!std::isfinite(out.scale) || out.scale <= 1e-6) return false;
  } else {
    out.scale = 1.0;
  }
  out.t = mean_dst - out.scale * (out.R * mean_src);
  return true;
}

// RANSAC over correspondences: component overlaps are contaminated by
// tracks that were triangulated badly in one gauge or the other, and a
// single outlier drags a closed-form fit arbitrarily far.
bool RobustSimilarity(const std::vector<Eigen::Vector3d>& src,
                      const std::vector<Eigen::Vector3d>& dst,
                      double inlier_radius_m, int min_inliers, bool allow_scale,
                      Similarity& out, int& inlier_count) {
  const size_t n = src.size();
  inlier_count = 0;
  if (n < static_cast<size_t>(std::max(3, min_inliers))) return false;

  std::mt19937 rng(0xC0FFEEu);
  std::uniform_int_distribution<size_t> pick(0, n - 1);
  std::vector<int> best_inliers;

  // Adaptive iteration count. Component overlaps run at very low inlier
  // ratios — two rooms sharing a doorway measured 6.6% — and a fixed 256
  // draws only finds a clean 3-point sample there about 7% of the time,
  // i.e. the merge succeeds or fails by luck. Recompute the requirement
  // from the best ratio seen so far (p = 0.999) and stop as soon as it is
  // met, so well-overlapped components still finish in a few hundred draws.
  constexpr int kMaxIterations = 40000;
  int needed = kMaxIterations;
  for (int iter = 0; iter < kMaxIterations && iter < needed; ++iter) {
    size_t i0 = pick(rng), i1 = pick(rng), i2 = pick(rng);
    if (i0 == i1 || i1 == i2 || i0 == i2) continue;
    Similarity candidate;
    if (!SimilarityFromCorrespondences({src[i0], src[i1], src[i2]},
                                       {dst[i0], dst[i1], dst[i2]}, candidate,
                                       allow_scale)) {
      continue;
    }
    std::vector<int> inliers;
    for (size_t i = 0; i < n; ++i) {
      if ((candidate.Apply(src[i]) - dst[i]).norm() < inlier_radius_m) {
        inliers.push_back(static_cast<int>(i));
      }
    }
    if (inliers.size() > best_inliers.size()) {
      best_inliers = std::move(inliers);
      const double w =
          static_cast<double>(best_inliers.size()) / static_cast<double>(n);
      const double p_sample = w * w * w;
      if (p_sample > 1e-9 && p_sample < 1.0) {
        const double est = std::log(1e-3) / std::log(1.0 - p_sample);
        needed = static_cast<int>(std::min<double>(kMaxIterations, est)) + 1;
      } else if (p_sample >= 1.0) {
        needed = iter + 1;
      }
    }
  }

  if (static_cast<int>(best_inliers.size()) < min_inliers) return false;

  // Refit on the consensus set, then re-collect (same LO-RANSAC pattern the
  // two-view estimator uses — the minimal-sample fit is biased).
  for (int pass = 0; pass < 2; ++pass) {
    std::vector<Eigen::Vector3d> s, d;
    s.reserve(best_inliers.size());
    d.reserve(best_inliers.size());
    for (const int i : best_inliers) {
      s.push_back(src[i]);
      d.push_back(dst[i]);
    }
    Similarity refined;
    if (!SimilarityFromCorrespondences(s, d, refined, allow_scale)) return false;
    out = refined;
    std::vector<int> recollected;
    for (size_t i = 0; i < n; ++i) {
      if ((out.Apply(src[i]) - dst[i]).norm() < inlier_radius_m) {
        recollected.push_back(static_cast<int>(i));
      }
    }
    if (recollected.size() < static_cast<size_t>(min_inliers)) break;
    best_inliers = std::move(recollected);
  }
  inlier_count = static_cast<int>(best_inliers.size());
  return inlier_count >= min_inliers;
}

// Moves a camera pose from a component's own gauge into the target gauge.
// The camera centre transforms like a point; the rotation composes with the
// similarity's rotation (scale never enters a pose's rotation block).
SE3 TransformPose(const SE3& pose_in_component, const Similarity& sim) {
  const Eigen::Quaterniond q_sim(sim.R);
  SE3 out;
  out.q = (pose_in_component.q * q_sim.conjugate()).normalized();
  const Eigen::Vector3d center_target = sim.Apply(pose_in_component.CameraCenter());
  out.t = -(out.q * center_target);
  return out;
}

}  // namespace bs

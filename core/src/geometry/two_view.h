#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "common/geometry.h"

namespace bs {

struct TwoViewOptions {
  double ransac_thresh_px = 1.5;
  double confidence = 0.9999;
  double min_cheirality_frac = 0.90;
  // Rotation-dominance is decided by actual parallax: the median
  // triangulation angle of the refined inliers. (An H-vs-E inlier ratio is
  // NOT usable here — indoor captures are wall-dominated, and a single
  // plane makes H explain everything even with a healthy baseline.)
  double min_median_tri_angle_deg = 0.4;
  // Compute the homography inlier count as a planarity diagnostic
  // (reported, never used for rejection).
  bool estimate_homography = true;
};

enum class TwoViewFailure {
  kNone = 0,
  kTooFewMatches,
  kEstimationFailed,
  kRotationDominant,   // H explains the pair: no reliable baseline
  kCheiralityFailed,
};

// Result of relative-pose estimation between two calibrated views.
// `rel_pose` maps camera-A coordinates to camera-B coordinates
// (x_b = R x_a + t) with unit-norm translation (scale is unobservable).
struct TwoViewResult {
  TwoViewFailure failure = TwoViewFailure::kEstimationFailed;
  SE3 rel_pose;
  std::vector<uint8_t> inlier_mask;  // per input correspondence
  int inliers_e = 0;
  int inliers_h = 0;                 // planarity diagnostic
  int cheirality_inliers = 0;
  double median_tri_angle_deg = 0;   // parallax of the refined inliers
  // Set when a homography explains (almost) all essential-matrix inliers:
  // the scene view is single-plane dominated and E carries the classic
  // conjugate-plane two-fold ambiguity — the returned pose may be the wrong
  // branch. Callers must not bootstrap from such pairs; registration via
  // PnP against existing structure resolves them instead.
  bool planar_ambiguous = false;

  bool ok() const { return failure == TwoViewFailure::kNone; }
};

// Estimates E (and H for the degeneracy guard) from pixel correspondences
// of two views sharing intrinsics K, recovers [R|t] with the cheirality
// test. Points must be undistorted pixel coordinates.
TwoViewResult EstimateRelativePose(const std::vector<cv::Point2f>& points_a,
                                   const std::vector<cv::Point2f>& points_b,
                                   const Intrinsics& K,
                                   const TwoViewOptions& options);

}  // namespace bs

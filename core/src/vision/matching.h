#pragma once

#include <vector>

#include "vision/features.h"

namespace bs {

struct Match {
  int idx_a = -1;   // feature index in image A
  int idx_b = -1;   // feature index in image B
  float distance = 0;
};

struct MatchOptions {
  float ratio = 0.8f;       // Lowe ratio threshold (<= 0 disables)
  bool cross_check = true;  // require mutual best match
  // Absolute descriptor-distance cap (<= 0 disables). Essential for guided
  // matching: without it a lone wrong keypoint inside the search radius
  // "wins" at arbitrary distance. Hamming units for ORB (64 is a sane cap),
  // L2 units for SIFT.
  float max_distance = 0.0f;
};

// Descriptor matching with ratio test + mutual cross-check. Distance metric
// follows the feature type (Hamming for ORB, L2 for SIFT). Both sets must
// share the same type.
std::vector<Match> MatchFeatures(const FeatureSet& a, const FeatureSet& b,
                                 const MatchOptions& options);

// Guided variant: accepts only candidates whose keypoint in B lies within
// `radius_px` of `predicted_b[idx_a]` (one prediction per A feature;
// prediction x<0 skips that feature). Used by the live tracker.
std::vector<Match> MatchFeaturesGuided(const FeatureSet& a, const FeatureSet& b,
                                       const std::vector<cv::Point2f>& predicted_b,
                                       float radius_px,
                                       const MatchOptions& options);

}  // namespace bs

#pragma once

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace bs {

enum class FeatureType { kOrb, kSift };

// Detected features of one image. Keypoint coordinates are in the pixel
// space of the image they were detected on (callers scale intrinsics to
// match). Descriptors: ORB = 32-byte binary rows, SIFT = 128-float rows
// (RootSIFT-normalized).
struct FeatureSet {
  FeatureType type = FeatureType::kOrb;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;

  int size() const { return static_cast<int>(keypoints.size()); }
};

// Detectors retry with a more sensitive threshold whenever a frame comes
// back short of this fraction of its feature budget. Motion blur and sensor
// noise flatten local contrast, so a fixed threshold silently starves real
// frames — measured on blurred captures, the retry recovers 4-5x the
// keypoints. Set high enough that well-textured frames never pay for a
// second detection pass.
inline constexpr float kFeatureRetryYieldFraction = 0.75f;

struct OrbOptions {
  int max_features = 1200;
  int levels = 8;
  float scale_factor = 1.2f;
  int fast_threshold = 20;
  int fast_threshold_min = 7;  // retry threshold on low-texture frames
  float retry_yield_frac = kFeatureRetryYieldFraction;
  // Spread features with a coarse grid occupancy cap so blank regions don't
  // starve while textured regions saturate.
  int grid_cols = 8;
  int grid_rows = 6;
};

struct SiftOptions {
  int max_features = 4096;
  bool root_sift = true;
  // OpenCV's default contrast threshold discards most keypoints on blurred
  // or noisy frames; the retry threshold recovers them.
  double contrast_threshold = 0.04;
  double contrast_threshold_min = 0.02;
  float retry_yield_frac = kFeatureRetryYieldFraction;
};

// Detects ORB features on an 8-bit gray image; retries with the low FAST
// threshold when the frame yields less than `retry_yield_frac` of budget.
FeatureSet DetectOrb(const cv::Mat& gray, const OrbOptions& options);

// Detects (Root)SIFT features; retries at the lower contrast threshold when
// the frame yields less than `retry_yield_frac` of budget.
FeatureSet DetectSift(const cv::Mat& gray, const SiftOptions& options);

}  // namespace bs

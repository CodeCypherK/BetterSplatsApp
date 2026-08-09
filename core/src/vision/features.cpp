#include "vision/features.h"

#include <algorithm>

namespace bs {

namespace {

// Keeps at most `cap` strongest keypoints per grid cell (with descriptors).
void GridFilter(FeatureSet& set, int cols, int rows, int max_total,
                int img_w, int img_h) {
  if (set.size() <= max_total) return;
  const float cell_w = static_cast<float>(img_w) / cols;
  const float cell_h = static_cast<float>(img_h) / rows;
  const int cap = std::max(1, max_total / (cols * rows));

  std::vector<int> order(set.keypoints.size());
  for (size_t i = 0; i < order.size(); ++i) order[i] = static_cast<int>(i);
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    return set.keypoints[a].response > set.keypoints[b].response;
  });

  std::vector<int> cell_counts(static_cast<size_t>(cols) * rows, 0);
  std::vector<int> kept;
  kept.reserve(max_total);
  for (const int idx : order) {
    const auto& kp = set.keypoints[idx];
    const int cx = std::min(cols - 1, static_cast<int>(kp.pt.x / cell_w));
    const int cy = std::min(rows - 1, static_cast<int>(kp.pt.y / cell_h));
    int& count = cell_counts[static_cast<size_t>(cy) * cols + cx];
    if (count >= cap) continue;
    ++count;
    kept.push_back(idx);
    if (static_cast<int>(kept.size()) >= max_total) break;
  }
  std::sort(kept.begin(), kept.end());

  std::vector<cv::KeyPoint> keypoints;
  keypoints.reserve(kept.size());
  cv::Mat descriptors(static_cast<int>(kept.size()), set.descriptors.cols,
                      set.descriptors.type());
  for (size_t i = 0; i < kept.size(); ++i) {
    keypoints.push_back(set.keypoints[kept[i]]);
    set.descriptors.row(kept[i]).copyTo(descriptors.row(static_cast<int>(i)));
  }
  set.keypoints = std::move(keypoints);
  set.descriptors = descriptors;
}

}  // namespace

FeatureSet DetectOrb(const cv::Mat& gray, const OrbOptions& options) {
  FeatureSet set;
  set.type = FeatureType::kOrb;

  // Over-detect, then grid-filter for spatial spread.
  auto orb = cv::ORB::create(options.max_features * 2, options.scale_factor,
                             options.levels, /*edgeThreshold=*/19, 0, 2,
                             cv::ORB::HARRIS_SCORE, 31, options.fast_threshold);
  orb->detectAndCompute(gray, cv::noArray(), set.keypoints, set.descriptors);

  if (set.size() < options.max_features / 3 &&
      options.fast_threshold_min < options.fast_threshold) {
    auto retry = cv::ORB::create(options.max_features * 2, options.scale_factor,
                                 options.levels, 19, 0, 2, cv::ORB::HARRIS_SCORE,
                                 31, options.fast_threshold_min);
    retry->detectAndCompute(gray, cv::noArray(), set.keypoints, set.descriptors);
  }

  GridFilter(set, options.grid_cols, options.grid_rows, options.max_features,
             gray.cols, gray.rows);
  return set;
}

FeatureSet DetectSift(const cv::Mat& gray, const SiftOptions& options) {
  FeatureSet set;
  set.type = FeatureType::kSift;
  auto sift = cv::SIFT::create(options.max_features);
  sift->detectAndCompute(gray, cv::noArray(), set.keypoints, set.descriptors);

  if (options.root_sift && !set.descriptors.empty()) {
    // RootSIFT: L1-normalize then sqrt — Hellinger kernel matching with
    // plain L2 distance. Standard accuracy boost for SfM.
    for (int r = 0; r < set.descriptors.rows; ++r) {
      cv::Mat row = set.descriptors.row(r);
      const double l1 = cv::norm(row, cv::NORM_L1);
      if (l1 > 0) {
        row /= static_cast<float>(l1);
        cv::sqrt(row, row);
      }
    }
  }
  return set;
}

}  // namespace bs

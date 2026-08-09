#include "vision/matching.h"

#include <cmath>

namespace bs {

namespace {

int NormFor(FeatureType type) {
  return type == FeatureType::kOrb ? cv::NORM_HAMMING : cv::NORM_L2;
}

// knn=2 forward matches with ratio filter; returns best index per query or
// -1. `second_best` gates the ratio test when there are >= 2 candidates.
std::vector<int> RatioMatch(const cv::Mat& query, const cv::Mat& train,
                            int norm, float ratio) {
  std::vector<int> best(query.rows, -1);
  if (query.empty() || train.empty()) return best;

  cv::BFMatcher matcher(norm, /*crossCheck=*/false);
  std::vector<std::vector<cv::DMatch>> knn;
  matcher.knnMatch(query, train, knn, 2);
  for (int i = 0; i < static_cast<int>(knn.size()); ++i) {
    if (knn[i].empty()) continue;
    if (ratio > 0 && knn[i].size() >= 2 &&
        knn[i][0].distance >= ratio * knn[i][1].distance) {
      continue;
    }
    best[i] = knn[i][0].trainIdx;
  }
  return best;
}

}  // namespace

std::vector<Match> MatchFeatures(const FeatureSet& a, const FeatureSet& b,
                                 const MatchOptions& options) {
  std::vector<Match> matches;
  if (a.descriptors.empty() || b.descriptors.empty() || a.type != b.type) {
    return matches;
  }
  const int norm = NormFor(a.type);

  const std::vector<int> forward =
      RatioMatch(a.descriptors, b.descriptors, norm, options.ratio);
  std::vector<int> backward;
  if (options.cross_check) {
    backward = RatioMatch(b.descriptors, a.descriptors, norm, options.ratio);
  }

  for (int ia = 0; ia < static_cast<int>(forward.size()); ++ia) {
    const int ib = forward[ia];
    if (ib < 0) continue;
    if (options.cross_check && backward[ib] != ia) continue;
    Match m;
    m.idx_a = ia;
    m.idx_b = ib;
    m.distance = static_cast<float>(
        cv::norm(a.descriptors.row(ia), b.descriptors.row(ib), norm));
    if (options.max_distance > 0 && m.distance > options.max_distance) continue;
    matches.push_back(m);
  }
  return matches;
}

std::vector<Match> MatchFeaturesGuided(const FeatureSet& a, const FeatureSet& b,
                                       const std::vector<cv::Point2f>& predicted_b,
                                       float radius_px,
                                       const MatchOptions& options) {
  std::vector<Match> matches;
  if (a.descriptors.empty() || b.descriptors.empty() || a.type != b.type ||
      predicted_b.size() != a.keypoints.size()) {
    return matches;
  }
  const int norm = NormFor(a.type);
  const float radius_sq = radius_px * radius_px;

  // Coarse hash grid over B keypoints for radius queries.
  const float cell = std::max(8.0f, radius_px);
  auto cell_key = [&](float x, float y) {
    return std::make_pair(static_cast<int>(x / cell), static_cast<int>(y / cell));
  };
  std::vector<std::vector<int>> grid;
  std::vector<std::pair<int, int>> grid_keys;
  auto find_cell = [&](int gx, int gy) -> const std::vector<int>* {
    for (size_t i = 0; i < grid_keys.size(); ++i) {
      if (grid_keys[i].first == gx && grid_keys[i].second == gy) return &grid[i];
    }
    return nullptr;
  };
  // Small maps beat unordered_map here; B has ~1-3k features spread over
  // ~200 cells, and this path runs at frame rate.
  for (int ib = 0; ib < static_cast<int>(b.keypoints.size()); ++ib) {
    const auto key = cell_key(b.keypoints[ib].pt.x, b.keypoints[ib].pt.y);
    bool placed = false;
    for (size_t i = 0; i < grid_keys.size(); ++i) {
      if (grid_keys[i] == key) {
        grid[i].push_back(ib);
        placed = true;
        break;
      }
    }
    if (!placed) {
      grid_keys.push_back(key);
      grid.push_back({ib});
    }
  }

  for (int ia = 0; ia < static_cast<int>(a.keypoints.size()); ++ia) {
    const cv::Point2f pred = predicted_b[ia];
    if (pred.x < 0) continue;

    int best_idx = -1;
    float best_dist = std::numeric_limits<float>::max();
    float second_dist = std::numeric_limits<float>::max();

    const auto center = cell_key(pred.x, pred.y);
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const auto* bucket = find_cell(center.first + dx, center.second + dy);
        if (bucket == nullptr) continue;
        for (const int ib : *bucket) {
          const cv::Point2f d = b.keypoints[ib].pt - pred;
          if (d.x * d.x + d.y * d.y > radius_sq) continue;
          const float dist = static_cast<float>(
              cv::norm(a.descriptors.row(ia), b.descriptors.row(ib), norm));
          if (dist < best_dist) {
            second_dist = best_dist;
            best_dist = dist;
            best_idx = ib;
          } else if (dist < second_dist) {
            second_dist = dist;
          }
        }
      }
    }

    if (best_idx < 0) continue;
    if (options.max_distance > 0 && best_dist > options.max_distance) continue;
    if (options.ratio > 0 && second_dist < std::numeric_limits<float>::max() &&
        best_dist >= options.ratio * second_dist) {
      continue;
    }
    Match m;
    m.idx_a = ia;
    m.idx_b = best_idx;
    m.distance = best_dist;
    matches.push_back(m);
  }
  return matches;
}

}  // namespace bs

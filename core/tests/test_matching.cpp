#include <gtest/gtest.h>

#include <random>

#include <opencv2/imgproc.hpp>

#include "vision/features.h"
#include "vision/matching.h"

namespace bs {
namespace {

// Builds a controlled matching problem: B contains bit-flipped copies of
// A's descriptors under a known permutation plus pure-noise distractors.
struct ControlledSets {
  FeatureSet a, b;
  std::vector<int> a_to_b;  // ground-truth mapping
};

ControlledSets MakeControlled(int n, int distractors, int flip_bits,
                              uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> byte(0, 255);
  std::uniform_int_distribution<int> bit(0, 255);
  std::uniform_real_distribution<float> coord(20.0f, 900.0f);

  ControlledSets out;
  out.a.type = out.b.type = FeatureType::kOrb;
  out.a.descriptors.create(n, 32, CV_8U);
  for (int i = 0; i < n; ++i) {
    for (int c = 0; c < 32; ++c) {
      out.a.descriptors.at<uint8_t>(i, c) = static_cast<uint8_t>(byte(rng));
    }
    out.a.keypoints.emplace_back(cv::Point2f(coord(rng), coord(rng)), 7.0f);
  }

  const int total_b = n + distractors;
  out.b.descriptors.create(total_b, 32, CV_8U);
  out.a_to_b.resize(n);
  std::vector<int> slots(total_b);
  for (int i = 0; i < total_b; ++i) slots[i] = i;
  std::shuffle(slots.begin(), slots.end(), rng);

  for (int i = 0; i < total_b; ++i) out.b.keypoints.emplace_back(
      cv::Point2f(coord(rng), coord(rng)), 7.0f);

  for (int i = 0; i < n; ++i) {
    const int slot = slots[i];
    out.a_to_b[i] = slot;
    out.a.descriptors.row(i).copyTo(out.b.descriptors.row(slot));
    for (int f = 0; f < flip_bits; ++f) {
      const int pos = bit(rng);
      out.b.descriptors.at<uint8_t>(slot, pos / 8) ^=
          static_cast<uint8_t>(1u << (pos % 8));
    }
  }
  for (int i = n; i < total_b; ++i) {
    const int slot = slots[i];
    for (int c = 0; c < 32; ++c) {
      out.b.descriptors.at<uint8_t>(slot, c) = static_cast<uint8_t>(byte(rng));
    }
  }
  return out;
}

TEST(Matching, RecoversKnownCorrespondences) {
  const ControlledSets sets = MakeControlled(200, 100, 6, 3);
  const std::vector<Match> matches = MatchFeatures(sets.a, sets.b, {});

  int correct = 0;
  for (const auto& m : matches) {
    if (sets.a_to_b[m.idx_a] == m.idx_b) ++correct;
  }
  EXPECT_GT(matches.size(), 180u);
  EXPECT_GT(static_cast<double>(correct) / matches.size(), 0.98);
}

TEST(Matching, CrossCheckKillsAsymmetricMatches) {
  // Duplicate one A descriptor many times in B: without cross-check it
  // matches somewhere; ratio + mutual consistency must reject it.
  ControlledSets sets = MakeControlled(50, 0, 0, 9);
  FeatureSet b2 = sets.b;
  for (int i = 0; i < 10; ++i) {
    cv::Mat extra;
    cv::vconcat(b2.descriptors, sets.a.descriptors.row(0), extra);
    b2.descriptors = extra;
    b2.keypoints.emplace_back(cv::Point2f(10.0f * i, 10.0f), 7.0f);
  }
  const std::vector<Match> matches = MatchFeatures(sets.a, b2, {});
  for (const auto& m : matches) {
    EXPECT_NE(m.idx_a, 0) << "ambiguous descriptor should not match";
  }
}

TEST(Matching, GuidedMatchRespectsRadius) {
  const ControlledSets sets = MakeControlled(150, 50, 4, 17);

  // Predictions: true keypoint position for even indices, far away for odd.
  std::vector<cv::Point2f> predictions(sets.a.keypoints.size(), {-1, -1});
  for (size_t i = 0; i < predictions.size(); ++i) {
    const cv::Point2f truth = sets.b.keypoints[sets.a_to_b[i]].pt;
    predictions[i] = (i % 2 == 0) ? truth + cv::Point2f(3.0f, -2.0f)
                                  : truth + cv::Point2f(500.0f, 500.0f);
  }

  MatchOptions options;
  options.max_distance = 64.0f;  // Hamming cap: random 256-bit pairs sit ~128
  const std::vector<Match> matches =
      MatchFeaturesGuided(sets.a, sets.b, predictions, 15.0f, options);
  int even_correct = 0;
  for (const auto& m : matches) {
    EXPECT_EQ(m.idx_a % 2, 0) << "far prediction must not match";
    if (sets.a_to_b[m.idx_a] == m.idx_b) ++even_correct;
  }
  EXPECT_GT(even_correct, 60);
}

TEST(Matching, MismatchedTypesReturnEmpty) {
  ControlledSets sets = MakeControlled(10, 0, 0, 1);
  FeatureSet sift = sets.b;
  sift.type = FeatureType::kSift;
  EXPECT_TRUE(MatchFeatures(sets.a, sift, {}).empty());
}

TEST(Features, OrbOnSyntheticTextureIsRepeatable) {
  // Same image twice -> identical detections; spread across the grid.
  cv::Mat img(720, 960, CV_8UC1);
  cv::RNG cvrng(42);
  cvrng.fill(img, cv::RNG::UNIFORM, 0, 255);
  cv::GaussianBlur(img, img, cv::Size(5, 5), 1.2);

  const FeatureSet f1 = DetectOrb(img, {});
  const FeatureSet f2 = DetectOrb(img, {});
  ASSERT_GT(f1.size(), 400);
  ASSERT_EQ(f1.size(), f2.size());
  for (int i = 0; i < f1.size(); ++i) {
    EXPECT_EQ(f1.keypoints[i].pt, f2.keypoints[i].pt);
  }

  // Grid spread: no cell holds more than a quarter of all features.
  std::vector<int> cells(48, 0);
  for (const auto& kp : f1.keypoints) {
    const int cx = std::min(7, static_cast<int>(kp.pt.x / (960.0f / 8)));
    const int cy = std::min(5, static_cast<int>(kp.pt.y / (720.0f / 6)));
    ++cells[cy * 8 + cx];
  }
  const int max_cell = *std::max_element(cells.begin(), cells.end());
  EXPECT_LT(max_cell, f1.size() / 4);
}

TEST(Features, SiftRootNormalization) {
  cv::Mat img(360, 480, CV_8UC1);
  cv::RNG cvrng(7);
  cvrng.fill(img, cv::RNG::UNIFORM, 0, 255);
  cv::GaussianBlur(img, img, cv::Size(5, 5), 1.5);

  const FeatureSet sift = DetectSift(img, {});
  ASSERT_GT(sift.size(), 50);
  // RootSIFT rows are L2-unit (sqrt of L1-normalized).
  for (int r = 0; r < std::min(20, sift.size()); ++r) {
    EXPECT_NEAR(cv::norm(sift.descriptors.row(r), cv::NORM_L2), 1.0, 1e-3);
  }
}

}  // namespace
}  // namespace bs

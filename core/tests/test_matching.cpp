#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include <opencv2/imgproc.hpp>

#include "common/geometry.h"
#include "synth_scene.h"
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

// A textured frame with a rectangle of it excluded — the shape of a person
// standing in a capture. Both detectors must return nothing from inside the
// excluded region, because a keypoint there becomes a track, and a track on a
// subject that moves between frames pulls on every camera pose that saw it.
TEST(Features, MaskExcludesKeypointsInsideIt) {
  cv::Mat img(720, 960, CV_8UC1);
  cv::RNG cvrng(7);
  cvrng.fill(img, cv::RNG::UNIFORM, 0, 255);
  cv::GaussianBlur(img, img, cv::Size(5, 5), 1.2);

  // Non-zero keeps, zero excludes (COLMAP convention, see features.h).
  const cv::Rect excluded(300, 200, 360, 300);
  cv::Mat mask(img.size(), CV_8UC1, cv::Scalar(255));
  mask(excluded).setTo(cv::Scalar(0));

  // The region really is feature-rich when nothing is masking it, otherwise
  // this test would pass on an image that simply had nothing there.
  int unmasked_inside = 0;
  for (const auto& kp : DetectOrb(img, {}).keypoints) {
    if (excluded.contains(cv::Point(kp.pt))) ++unmasked_inside;
  }
  ASSERT_GT(unmasked_inside, 20);

  for (const auto& kp : DetectOrb(img, {}, mask).keypoints) {
    EXPECT_FALSE(excluded.contains(cv::Point(kp.pt))) << "ORB kept a masked pixel";
  }
  SiftOptions sift;
  sift.max_features = 1500;
  for (const auto& kp : DetectSift(img, sift, mask).keypoints) {
    EXPECT_FALSE(excluded.contains(cv::Point(kp.pt))) << "SIFT kept a masked pixel";
  }
}

// Absent means "no opinion", never "exclude everything" — the property that
// lets masks be an optional derived layer that no existing session notices.
TEST(Features, EmptyMaskMatchesTheUnmaskedPath) {
  cv::Mat img(480, 640, CV_8UC1);
  cv::RNG cvrng(11);
  cvrng.fill(img, cv::RNG::UNIFORM, 0, 255);
  cv::GaussianBlur(img, img, cv::Size(5, 5), 1.2);

  const FeatureSet plain = DetectOrb(img, {});
  const FeatureSet empty_mask = DetectOrb(img, {}, cv::Mat());
  ASSERT_EQ(plain.size(), empty_mask.size());
  for (int i = 0; i < plain.size(); ++i) {
    EXPECT_EQ(plain.keypoints[i].pt, empty_mask.keypoints[i].pt);
  }
}

// The retry gate treats "short of budget" as a starved frame and re-detects at
// a more sensitive threshold. A masked frame is short for an entirely
// different reason — the excluded part has no features to give — so the gate
// has to be scaled by how much of the frame survives. Without that scaling,
// masking half an image makes every frame retry, and the sensitive pass
// sprays weaker keypoints across the half that was never the problem: a mask
// would ADD noise features to the region it was supposed to leave alone.
TEST(Features, MaskDoesNotProvokeTheStarvedFrameRetry) {
  cv::Mat img(720, 960, CV_8UC1);
  cv::RNG cvrng(23);
  cvrng.fill(img, cv::RNG::UNIFORM, 0, 255);
  cv::GaussianBlur(img, img, cv::Size(5, 5), 1.2);

  const cv::Rect visible(0, 0, 480, 720);  // keep the left half
  cv::Mat mask(img.size(), CV_8UC1, cv::Scalar(0));
  mask(visible).setTo(cv::Scalar(255));

  auto count_in = [&](const FeatureSet& set) {
    int n = 0;
    for (const auto& kp : set.keypoints) {
      if (visible.contains(cv::Point(kp.pt))) ++n;
    }
    return n;
  };

  const int plain_left = count_in(DetectOrb(img, {}));
  const int masked_left = count_in(DetectOrb(img, {}, mask));
  ASSERT_GT(plain_left, 100);
  // Masking the other half must not densify this one. The retry it would
  // otherwise trigger detects at fast_threshold_min and roughly doubles the
  // yield here, so 1.5x separates "no retry" from "retried" comfortably.
  EXPECT_LT(masked_left, plain_left * 3 / 2);
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

// Blur and sensor noise flatten local contrast, which starves a fixed
// detector threshold long before the scene runs out of structure. Both
// detectors must notice the shortfall and re-detect more sensitively.
// Regression guard: the retry gate used to sit so low (1/3 of budget) that
// a frame at ~46% of budget kept a 4x-thinner feature set than it could.
TEST(Features, DetectorsRecoverYieldOnDegradedFrames) {
  // Render the same real scene view twice: once clean, once with the motion
  // blur and sensor noise of a handheld capture. Random-noise images cannot
  // show this — they are saturated with contrast and never starve.
  const synth::Scene scene = synth::MakeRoomScene(3);
  const std::vector<SE3> poses =
      synth::OrbitTrajectory(8, 1.6, 2.2, 1.5, 3, 60.0);
  Intrinsics K;
  K.width = 640;
  K.height = 480;
  K.fx = K.fy = 0.5 * 640 / std::tan(DegToRad(70.0) / 2.0);
  K.cx = 0.5 * (640 - 1);
  K.cy = 0.5 * (480 - 1);

  synth::RenderOptions hard;
  hard.motion_blur_px = 9.0f;
  hard.motion_blur_angle = 0.2;
  hard.noise_sigma = 4.0f;

  cv::Mat sharp_bgr = synth::RenderImage(scene, poses[4], K);
  cv::Mat degraded_bgr = synth::RenderImage(scene, poses[4], K, hard);
  cv::Mat sharp, degraded;
  cv::cvtColor(sharp_bgr, sharp, cv::COLOR_BGR2GRAY);
  cv::cvtColor(degraded_bgr, degraded, cv::COLOR_BGR2GRAY);

  OrbOptions orb_no_retry;
  orb_no_retry.retry_yield_frac = 0.0f;  // disable: what a fixed gate gives
  const int orb_fixed = DetectOrb(degraded, orb_no_retry).size();
  const int orb_adaptive = DetectOrb(degraded, {}).size();
  EXPECT_GT(orb_adaptive, orb_fixed)
      << "ORB retry did not fire on a degraded frame";

  SiftOptions sift_no_retry;
  sift_no_retry.max_features = 3000;
  sift_no_retry.retry_yield_frac = 0.0f;
  SiftOptions sift_adaptive;
  sift_adaptive.max_features = 3000;
  const int sift_fixed = DetectSift(degraded, sift_no_retry).size();
  const int sift_adaptive_n = DetectSift(degraded, sift_adaptive).size();
  EXPECT_GT(sift_adaptive_n, sift_fixed)
      << "SIFT retry did not fire on a degraded frame";

  // A well-textured frame already fills its budget, so it must NOT pay for
  // a second detection pass — identical results with and without the retry.
  EXPECT_EQ(DetectOrb(sharp, {}).size(), DetectOrb(sharp, orb_no_retry).size());
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

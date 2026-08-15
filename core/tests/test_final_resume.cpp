// Final-solve resume: features/matches persist to final/cache/ and a second
// run reloads them; config or session changes invalidate wholesale.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

#include "common/config.h"
#include "common/geometry.h"
#include "final/final_solve.h"
#include "generate.h"
#include "io/session_reader.h"

namespace bs {
namespace {

namespace fs = std::filesystem;

class FinalResumeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Per-test directory: these two cases both build a session and a cache,
    // so a shared path makes them pass only when ctest runs serially.
    dir_ = (fs::temp_directory_path() /
            (std::string("bs_final_resume_") +
             ::testing::UnitTest::GetInstance()->current_test_info()->name()))
               .string();
    fs::remove_all(dir_);

    synth::GenerateOptions options;
    options.out_dir = dir_;
    options.frame_count = 14;
    options.image_width = 480;
    options.image_height = 360;
    options.depth_width = 160;
    options.depth_height = 120;
    options.seed = 9;
    options.sweep_deg = 30.0;
    ASSERT_TRUE(synth::GenerateSession(options));

    // Final solve initializes from live poses; fabricate them from ground
    // truth (the live pipeline is exercised elsewhere).
    const auto session = SessionReader::Open(dir_);
    ASSERT_TRUE(session.has_value());
    const auto gt = session->ReadGroundTruth();
    ASSERT_TRUE(gt.has_value());
    fs::create_directories(fs::path(dir_) / "live");
    std::ofstream poses(fs::path(dir_) / "live" / "poses.jsonl");
    poses.precision(12);
    for (const auto& p : gt->poses) {
      poses << "{\"frame_id\":" << p.frame_id << ",\"t\":0.0,"
            << "\"state\":\"tracking\",\"q\":[" << p.q[0] << "," << p.q[1]
            << "," << p.q[2] << "," << p.q[3] << "],\"p\":[" << p.t[0] << ","
            << p.t[1] << "," << p.t[2] << "]}\n";
    }
  }

  void TearDown() override { fs::remove_all(dir_); }

  FinalOutcome Solve() {
    EngineConfig config;
    config.log_level = 3;  // errors only; keep test output clean
    return RunFinalSolve(config, dir_, "fast", nullptr, nullptr);
  }

  std::string dir_;
};

TEST_F(FinalResumeTest, SecondRunReusesCacheWithIdenticalResults) {
  const FinalOutcome first = Solve();
  ASSERT_TRUE(first.ok) << first.error;
  EXPECT_EQ(first.metrics.features_cached, 0u);
  EXPECT_EQ(first.metrics.matches_cached, 0u);
  EXPECT_GE(first.metrics.images_registered, 12u);
  EXPECT_GT(first.metrics.points, 200u);
  // A complete live pass IS a usable initialization, and is used as one.
  EXPECT_EQ(first.metrics.live_poses_used, first.metrics.images_total);

  const FinalOutcome second = Solve();
  ASSERT_TRUE(second.ok) << second.error;
  EXPECT_EQ(second.metrics.features_cached, second.metrics.images_total);
  EXPECT_GT(second.metrics.matches_cached, 0u);
  EXPECT_EQ(second.metrics.images_registered, first.metrics.images_registered);
  // Same inputs + same deterministic pipeline stages -> same structure.
  EXPECT_NEAR(static_cast<double>(second.metrics.points),
              static_cast<double>(first.metrics.points),
              0.05 * first.metrics.points + 1);
  EXPECT_FLOAT_EQ(second.metrics.mean_track_len, first.metrics.mean_track_len);
}

TEST_F(FinalResumeTest, ManifestMismatchForcesRecompute) {
  ASSERT_TRUE(Solve().ok);
  std::ofstream(fs::path(dir_) / "final" / "cache" / "manifest.txt")
      << "1234567890\n";

  const FinalOutcome redo = Solve();
  ASSERT_TRUE(redo.ok) << redo.error;
  EXPECT_EQ(redo.metrics.features_cached, 0u);
  EXPECT_EQ(redo.metrics.matches_cached, 0u);
}

// A live pass that bootstrapped and immediately lost tracking leaves a
// handful of poses behind, and those are worse than none: the solve seeds
// itself on two frames that happen to be wherever tracking started, in
// whatever scale the live gauge locked, instead of choosing a
// well-conditioned pair from image geometry.
//
// Measured on a 340-frame two-room capture: two live poses out of 340, with
// the scale lock 2.7x out, finished at 6/340 registered and 1,573 points.
// Discarding them took the same session to 99/340 and 26,734 points, and
// ground-truth error from 7.0 m to 5.4 cm. "At least two poses" is not a
// usable initialization; a fraction of the session is.
TEST_F(FinalResumeTest, TwoLivePosesAreDiscardedRatherThanSeededFrom) {
  const auto session = SessionReader::Open(dir_);
  ASSERT_TRUE(session.has_value());
  const auto gt = session->ReadGroundTruth();
  ASSERT_TRUE(gt.has_value());
  ASSERT_GE(gt->poses.size(), 3u);

  // Keep the first two ground-truth poses and shrink them by the same
  // factor, which is what a bad live scale lock looks like: self-consistent,
  // metrically wrong, and enough to poison everything triangulated off it.
  std::ofstream poses(fs::path(dir_) / "live" / "poses.jsonl",
                      std::ios::trunc);
  poses.precision(12);
  for (int i = 0; i < 2; ++i) {
    const auto& p = gt->poses[i];
    poses << "{\"frame_id\":" << p.frame_id << ",\"t\":0.0,"
          << "\"state\":\"tracking\",\"q\":[" << p.q[0] << "," << p.q[1] << ","
          << p.q[2] << "," << p.q[3] << "],\"p\":[" << 0.37 * p.t[0] << ","
          << 0.37 * p.t[1] << "," << 0.37 * p.t[2] << "]}\n";
  }
  poses.close();

  const FinalOutcome out = Solve();
  ASSERT_TRUE(out.ok) << out.error;
  // The decision itself, which is what this test is about — 14 mutually
  // visible frames recover from a bad seed either way, so the OUTCOME does
  // not separate the two behaviours and the metric has to.
  EXPECT_EQ(out.metrics.live_poses_used, 0u)
      << "solve seeded itself on the thin live hint instead of bootstrapping";
  EXPECT_GE(out.metrics.images_registered, 12u);
  EXPECT_GT(out.metrics.points, 200u);
  // And the model is metric, not 0.37x. A solve that adopted the live gauge
  // would be internally consistent and 2.7x too small — which neither the
  // registration count nor the point count would show. How far the camera
  // actually travelled is the check, read back out of the export.
  auto widest = [](const std::vector<Eigen::Vector3d>& c) {
    double worst = 0;
    for (const auto& a : c) {
      for (const auto& b : c) worst = std::max(worst, (a - b).norm());
    }
    return worst;
  };

  std::vector<Eigen::Vector3d> truth;
  for (const auto& p : gt->poses) {
    SE3 pose;
    pose.q = Eigen::Quaterniond(p.q[0], p.q[1], p.q[2], p.q[3]);
    pose.t = Eigen::Vector3d(p.t[0], p.t[1], p.t[2]);
    truth.push_back(pose.CameraCenter());
  }

  // images.txt: "ID QW QX QY QZ TX TY TZ CAM NAME" on every other line.
  std::ifstream images(fs::path(dir_) / "final" / "colmap" / "images.txt");
  ASSERT_TRUE(images.good());
  std::vector<Eigen::Vector3d> solved;
  std::string line;
  bool pose_line = true;
  while (std::getline(images, line)) {
    if (line.empty() || line[0] == '#') continue;
    if (!pose_line) {  // the observation line that follows each pose
      pose_line = true;
      continue;
    }
    pose_line = false;
    std::istringstream in(line);
    int id = 0, cam = 0;
    SE3 pose;
    in >> id >> pose.q.w() >> pose.q.x() >> pose.q.y() >> pose.q.z() >>
        pose.t.x() >> pose.t.y() >> pose.t.z() >> cam;
    if (!in) continue;
    solved.push_back(pose.CameraCenter());
  }
  ASSERT_GE(solved.size(), 12u);
  EXPECT_NEAR(widest(solved), widest(truth), 0.10 * widest(truth))
      << "solve adopted the live pass's wrong scale";
}

}  // namespace
}  // namespace bs

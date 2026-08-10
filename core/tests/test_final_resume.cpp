// Final-solve resume: features/matches persist to final/cache/ and a second
// run reloads them; config or session changes invalidate wholesale.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "common/config.h"
#include "final/final_solve.h"
#include "generate.h"
#include "io/session_reader.h"

namespace bs {
namespace {

namespace fs = std::filesystem;

class FinalResumeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = (fs::temp_directory_path() / "bs_final_resume").string();
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

}  // namespace
}  // namespace bs

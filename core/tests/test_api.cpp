// C ABI behavior: lifecycle, state machine, null-safety, selftest.

#include <gtest/gtest.h>

#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "bs/bs_api.h"

namespace {

namespace fs = std::filesystem;

class ApiTest : public ::testing::Test {
 protected:
  void SetUp() override {
    engine_ = bs_create("{}");
    ASSERT_NE(engine_, nullptr);
    session_dir_ =
        (fs::temp_directory_path() / "bs_api_test_session").string();
    fs::remove_all(session_dir_);
  }

  void TearDown() override {
    bs_destroy(engine_);
    fs::remove_all(session_dir_);
  }

  bs_frame_in MakeFrame(uint32_t id) {
    luma_.assign(64 * 48, 128);
    depth_.assign(16 * 12, 1.5f);
    bs_frame_in f{};
    f.frame_id = id;
    f.t_capture = 0.033 * id;
    f.t_depth = f.t_capture;
    f.luma = luma_.data();
    f.luma_width = 64;
    f.luma_height = 48;
    f.luma_stride = 64;
    f.depth = depth_.data();
    f.depth_width = 16;
    f.depth_height = 12;
    f.fx = f.fy = 50.0;
    f.cx = 32.0;
    f.cy = 24.0;
    f.dfx = f.dfy = 12.0;
    f.dcx = 8.0;
    f.dcy = 6.0;
    return f;
  }

  bs_engine* engine_ = nullptr;
  std::string session_dir_;
  std::vector<uint8_t> luma_;
  std::vector<float> depth_;
};

TEST(ApiBasics, VersionIsNonEmpty) {
  const char* v = bs_version();
  ASSERT_NE(v, nullptr);
  EXPECT_GT(std::strlen(v), 0u);
}

TEST(ApiBasics, NullSafety) {
  EXPECT_EQ(bs_live_begin(nullptr, "x", BS_PASS_CAPTURE), BS_ERR_INVALID_ARGUMENT);
  EXPECT_EQ(bs_live_feed(nullptr, nullptr), BS_ERR_INVALID_ARGUMENT);
  EXPECT_EQ(bs_live_poll_status(nullptr, nullptr), BS_ERR_INVALID_ARGUMENT);
  EXPECT_EQ(bs_final_start(nullptr, "x", "quality"), BS_ERR_INVALID_ARGUMENT);
  EXPECT_NE(bs_last_error(nullptr), nullptr);
  bs_destroy(nullptr);  // must not crash
  bs_snapshot_release(nullptr, nullptr);
}

TEST(ApiBasics, SelftestPassesOnHost) {
  char buf[512] = {0};
  const bs_result r = bs_selftest(buf, sizeof(buf));
  EXPECT_EQ(r, BS_OK) << buf;
  EXPECT_NE(std::strstr(buf, "eigen ok"), nullptr) << buf;
  EXPECT_NE(std::strstr(buf, "opencv ok"), nullptr) << buf;
  EXPECT_NE(std::strstr(buf, "ceres ok"), nullptr) << buf;
}

TEST_F(ApiTest, LiveLifecycle) {
  bs_live_status status{};
  ASSERT_EQ(bs_live_poll_status(engine_, &status), BS_OK);
  EXPECT_EQ(status.state, BS_LIVE_IDLE);

  ASSERT_EQ(bs_live_begin(engine_, session_dir_.c_str(), BS_PASS_CAPTURE), BS_OK);
  EXPECT_TRUE(fs::exists(fs::path(session_dir_) / "live"));

  for (uint32_t id = 1; id <= 5; ++id) {
    bs_frame_in f = MakeFrame(id);
    ASSERT_EQ(bs_live_feed(engine_, &f), BS_OK) << bs_last_error(engine_);
  }

  ASSERT_EQ(bs_live_poll_status(engine_, &status), BS_OK);
  EXPECT_EQ(status.state, BS_LIVE_INITIALIZING);
  EXPECT_EQ(status.frames_fed, 5u);
  EXPECT_EQ(status.last_frame_id, 5u);

  ASSERT_EQ(bs_live_end(engine_), BS_OK);
  ASSERT_EQ(bs_live_poll_status(engine_, &status), BS_OK);
  EXPECT_EQ(status.state, BS_LIVE_FINISHED);
}

TEST_F(ApiTest, LiveStateMachineRejectsBadTransitions) {
  bs_frame_in f = MakeFrame(1);
  EXPECT_EQ(bs_live_feed(engine_, &f), BS_ERR_INVALID_STATE);
  EXPECT_EQ(bs_live_end(engine_), BS_ERR_INVALID_STATE);

  ASSERT_EQ(bs_live_begin(engine_, session_dir_.c_str(), BS_PASS_CAPTURE), BS_OK);
  EXPECT_EQ(bs_live_begin(engine_, session_dir_.c_str(), BS_PASS_CAPTURE), BS_ERR_INVALID_STATE);

  // Non-increasing frame ids are rejected.
  f = MakeFrame(7);
  ASSERT_EQ(bs_live_feed(engine_, &f), BS_OK);
  f = MakeFrame(7);
  EXPECT_EQ(bs_live_feed(engine_, &f), BS_ERR_INVALID_ARGUMENT);
  EXPECT_NE(std::strlen(bs_last_error(engine_)), 0u);
}

TEST_F(ApiTest, SnapshotAcquireReleaseIsCleanWhenEmpty) {
  bs_snapshot snap{};
  ASSERT_EQ(bs_snapshot_acquire(engine_, &snap), BS_OK);
  EXPECT_EQ(snap.point_count, 0u);
  EXPECT_EQ(snap.camera_count, 0u);
  bs_snapshot_release(engine_, &snap);
}

TEST_F(ApiTest, ThermalHintValidation) {
  EXPECT_EQ(bs_thermal_hint(engine_, 0), BS_OK);
  EXPECT_EQ(bs_thermal_hint(engine_, 3), BS_OK);
  EXPECT_EQ(bs_thermal_hint(engine_, 4), BS_ERR_INVALID_ARGUMENT);
  EXPECT_EQ(bs_thermal_hint(engine_, -1), BS_ERR_INVALID_ARGUMENT);
}

}  // namespace

// Re-bootstrap after a hopeless loss, and the gauge boundary it creates.
//
// The live tracker abandons its map when it has been lost long enough that
// searching is pointless, and rebuilds from where the camera actually is.
// That rebuild starts a NEW WORLD FRAME: new origin, new scale. These tests
// pin the two things that must hold across that boundary — the boundary is
// recorded, and nothing from the dead gauge leaks into the new one — because
// both failures are silent. Poses from two frames average into a number that
// looks like tracking error, and an inherited metre looks like a locked scale.

#include <gtest/gtest.h>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "common/config.h"
#include "common/geometry.h"
#include "generate.h"
#include "io/session_reader.h"
#include "live/live_system.h"

namespace bs {
namespace {

namespace fs = std::filesystem;

struct LoggedPose {
  uint32_t frame_id = 0;
  uint32_t segment = 0;
  bool tracked = false;
  SE3 pose;
};

std::vector<LoggedPose> ReadPoseLog(const std::string& dir) {
  std::vector<LoggedPose> out;
  std::ifstream in(fs::path(dir) / "live" / "poses.jsonl");
  std::string line;
  while (std::getline(in, line)) {
    LoggedPose lp;
    double t = 0;
    if (std::sscanf(line.c_str(), "{\"frame_id\":%u,\"t\":%lf", &lp.frame_id,
                    &t) != 2) {
      continue;
    }
    if (const char* seg = std::strstr(line.c_str(), "\"segment\":")) {
      std::sscanf(seg, "\"segment\":%u", &lp.segment);
    }
    lp.tracked = line.find("\"tracking\"") != std::string::npos;
    if (lp.tracked) {
      double qw, qx, qy, qz, px, py, pz;
      const char* q = std::strstr(line.c_str(), "\"q\":[");
      const char* p = std::strstr(line.c_str(), "\"p\":[");
      if (q == nullptr || p == nullptr ||
          std::sscanf(q, "\"q\":[%lf,%lf,%lf,%lf]", &qw, &qx, &qy, &qz) != 4 ||
          std::sscanf(p, "\"p\":[%lf,%lf,%lf]", &px, &py, &pz) != 3) {
        continue;
      }
      lp.pose.q = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
      lp.pose.t = Eigen::Vector3d(px, py, pz);
    }
    out.push_back(lp);
  }
  return out;
}

class LiveSegmentTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = (fs::temp_directory_path() /
            (std::string("bs_live_seg_") +
             ::testing::UnitTest::GetInstance()->current_test_info()->name()))
               .string();
    fs::remove_all(dir_);

    synth::GenerateOptions options;
    options.out_dir = dir_;
    options.frame_count = 40;
    options.image_width = 480;
    options.image_height = 360;
    options.depth_width = 160;
    options.depth_height = 120;
    options.seed = 5;
    options.sweep_deg = 40.0;
    ASSERT_TRUE(synth::GenerateSession(options));

    session_ = SessionReader::Open(dir_);
    ASSERT_TRUE(session_.has_value());
    gt_ = session_->ReadGroundTruth();
    ASSERT_TRUE(gt_.has_value());
  }

  void TearDown() override { fs::remove_all(dir_); }

  // Feeds the session's frames, replacing a middle stretch with featureless
  // gray. The tracker cannot match anything there, so it loses tracking and
  // — with a short give-up budget — abandons the map partway through and
  // bootstraps a second one on the frames that follow.
  void Feed(LiveSystem& live, uint32_t blind_from, uint32_t blind_to) {
    for (const auto& id : session_->frame_ids()) {
      const auto meta = session_->ReadMeta(id);
      const auto depth = session_->ReadDepth(id);
      const auto bytes = session_->ReadImageBytes(id);
      ASSERT_TRUE(meta.has_value() && depth.has_value() && bytes.has_value())
          << "frame " << id;
      cv::Mat image = cv::imdecode(*bytes, cv::IMREAD_GRAYSCALE);
      ASSERT_FALSE(image.empty());

      LiveFrameInput input;
      input.frame_id = id;
      input.t_capture = meta->t_capture;
      input.t_depth = meta->t_depth;
      // Session intrinsics are recorded against ref_w/ref_h; the decoded
      // image is what the tracker actually sees.
      const auto& pin = meta->intrinsics;
      input.K = Intrinsics{pin.fx, pin.fy, pin.cx, pin.cy, pin.ref_w,
                           pin.ref_h}
                    .ScaledTo(image.cols, image.rows);
      const auto& dpin = meta->depth_intrinsics;
      input.Kd = Intrinsics{dpin.fx,    dpin.fy,   dpin.cx,
                            dpin.cy,    dpin.ref_w, dpin.ref_h};
      input.depth = *depth;
      if (id >= blind_from && id <= blind_to) {
        // Uniform gray: no corners, so no features, so no matches.
        input.gray = cv::Mat(image.rows, image.cols, CV_8UC1,
                             cv::Scalar(128));
      } else {
        input.gray = image;
      }
      live.Feed(input);
    }
    ASSERT_TRUE(live.End());
  }

  std::optional<SE3> GroundTruthPose(uint32_t frame_id) const {
    for (const auto& gp : gt_->poses) {
      if (gp.frame_id != frame_id) continue;
      SE3 pose;
      pose.q = Eigen::Quaterniond(gp.q[0], gp.q[1], gp.q[2], gp.q[3]);
      pose.t = Eigen::Vector3d(gp.t[0], gp.t[1], gp.t[2]);
      return pose;
    }
    return std::nullopt;
  }

  // Camera-center RMSE of one segment against ground truth, anchored at that
  // segment's own first pose. Scale is NOT re-fit — the whole point is that a
  // wrong metre must show up as distance error.
  double SegmentRmse(const std::vector<LoggedPose>& group) const {
    const auto ref = GroundTruthPose(group.front().frame_id);
    if (!ref) return -1.0;
    const SE3 align = group.front().pose.Inverse() * (*ref);
    double sq = 0;
    int n = 0;
    for (const auto& lp : group) {
      const auto gt_pose = GroundTruthPose(lp.frame_id);
      if (!gt_pose) continue;
      const SE3 in_live = *gt_pose * align.Inverse();
      sq += (lp.pose.CameraCenter() - in_live.CameraCenter()).squaredNorm();
      ++n;
    }
    return n > 0 ? std::sqrt(sq / n) : -1.0;
  }

  static std::map<uint32_t, std::vector<LoggedPose>> BySegment(
      const std::vector<LoggedPose>& log) {
    std::map<uint32_t, std::vector<LoggedPose>> out;
    for (const auto& lp : log) {
      if (lp.tracked) out[lp.segment].push_back(lp);
    }
    return out;
  }

  EngineConfig Config(int give_up) const {
    EngineConfig config;
    config.log_level = 3;  // errors only; keep test output clean
    config.live_relocalize_give_up_frames = give_up;
    return config;
  }

  std::string dir_;
  std::optional<SessionReader> session_;
  std::optional<GroundTruth> gt_;
};

// A run that never gets hopelessly lost must stay in one world frame. If this
// ever fires, the tracker is discarding maps it could still have used, and
// every downstream consumer that keeps "the largest segment" is silently
// throwing away most of the capture.
TEST_F(LiveSegmentTest, AHealthyRunStaysInOneSegment) {
  EngineConfig config = Config(300);
  LiveSystem live(config);
  live.Begin(dir_, 0.0, 0.0);
  Feed(live, /*blind_from=*/1, /*blind_to=*/0);  // empty range: nothing blind

  const auto log = ReadPoseLog(dir_);
  ASSERT_FALSE(log.empty());
  for (const auto& lp : log) {
    EXPECT_EQ(lp.segment, 0u) << "frame " << lp.frame_id;
  }
}

// Blind the tracker for longer than it is willing to search, and it must
// give up on that map rather than spend the rest of the capture looking for
// it. A real device session spent 4,380 consecutive frames in that search
// and mapped nothing; this is the test that would have caught it.
TEST_F(LiveSegmentTest, AHopelessLossStartsANewSegment) {
  EngineConfig config = Config(6);
  LiveSystem live(config);
  live.Begin(dir_, 0.0, 0.0);
  Feed(live, /*blind_from=*/14, /*blind_to=*/24);

  const auto by_segment = BySegment(ReadPoseLog(dir_));
  ASSERT_GE(by_segment.size(), 2u)
      << "tracker never abandoned a map it could not localize against";

  // Every segment is contiguous and they run in order: a segment id is a
  // point in time, not a label that can reappear.
  uint32_t previous_last = 0;
  for (const auto& [seg, group] : by_segment) {
    EXPECT_GT(group.front().frame_id, previous_last)
        << "segment " << seg << " overlaps the one before it";
    previous_last = group.back().frame_id;
  }
}

// The gauge test, and the reason this file exists. Scale is locked from
// d_lidar / z_est samples measured against a bootstrap's arbitrary two-view
// baseline. Those samples belong to the map that produced them; carried into
// the next bootstrap they outnumber its own and hand the new map the old
// map's metre. The failure is invisible — scale still reads "locked", poses
// still look smooth — and shows up only as a new segment whose distances are
// wrong by the ratio between two baselines.
TEST_F(LiveSegmentTest, ANewSegmentMeasuresItsOwnScale) {
  EngineConfig config = Config(6);
  LiveSystem live(config);
  live.Begin(dir_, 0.0, 0.0);
  Feed(live, /*blind_from=*/14, /*blind_to=*/24);

  const auto by_segment = BySegment(ReadPoseLog(dir_));
  ASSERT_GE(by_segment.size(), 2u);

  // Measured in its own frame, every segment with enough poses to measure is
  // metrically correct. A leaked scale shows up here as a proportional
  // distance error and nowhere else.
  int measured = 0;
  for (const auto& [seg, group] : by_segment) {
    if (group.size() < 5) continue;
    const double rmse = SegmentRmse(group);
    ASSERT_GE(rmse, 0.0) << "segment " << seg << " has no ground truth";
    EXPECT_LT(rmse, 0.05) << "segment " << seg << " drifts " << rmse
                          << " m from ground truth in its own world frame";
    ++measured;
  }
  EXPECT_GE(measured, 2) << "needs two measurable segments to be meaningful";
}

}  // namespace
}  // namespace bs

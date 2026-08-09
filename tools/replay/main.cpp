// bs_replay — run the engine against a captured session directory on Linux.
//
// This is the primary debugging path for the project: the iPhone app exports
// a full-session zip (RAW layer), and this CLI feeds those frames through the
// same engine binary logic that runs on the phone — live pipeline first, then
// the final solve. No Mac or device required to reproduce engine behavior.

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "bs/bs_api.h"
#include "io/session_reader.h"

namespace {

void PrintUsage() {
  std::fprintf(stderr,
               "usage: bs_replay <session_dir> [--info] [--live] [--config J]\n"
               "       bs_replay --version | --selftest\n"
               "  --info    print session summary and exit\n"
               "  --live    feed all frames through the live pipeline\n"
               "  --config  engine config JSON string (default {})\n");
}

int RunInfo(const bs::SessionReader& session) {
  const auto& info = session.info();
  std::printf("session      %s\n", info.session_id.c_str());
  std::printf("device       %s (iOS %s)\n", info.device_model.c_str(),
              info.device_ios.c_str());
  std::printf("video        %dx%d @%dfps (%s)\n", info.video_w, info.video_h,
              info.video_fps, info.video_pixel_format.c_str());
  std::printf("depth        %dx%d (%s, filtering=%s)\n", info.depth_w,
              info.depth_h, info.depth_format.c_str(),
              info.depth_filtering ? "on" : "off");
  std::printf("frames       %zu on disk (session.json says %u)\n",
              session.frame_ids().size(), info.frame_count);
  std::printf("keyframes    %zu\n", info.keyframe_ids.size());
  const auto gt = session.ReadGroundTruth();
  std::printf("ground truth %s\n", gt ? "present (synthetic session)" : "absent");

  if (!session.frame_ids().empty()) {
    const uint32_t first = session.frame_ids().front();
    const auto meta = session.ReadMeta(first);
    const auto depth = session.ReadDepth(first);
    if (meta) {
      std::printf("frame %06u  fx=%.1f cx=%.1f  lap_var=%.0f  kf=%d\n", first,
                  meta->intrinsics.fx, meta->intrinsics.cx,
                  meta->quality.lap_var, meta->is_keyframe ? 1 : 0);
    }
    if (depth) {
      int valid = 0;
      for (int y = 0; y < depth->height; ++y) {
        for (int x = 0; x < depth->width; ++x) {
          if (depth->ValidAt(x, y)) ++valid;
        }
      }
      std::printf("frame %06u  depth %dx%d, %.1f%% valid\n", first, depth->width,
                  depth->height,
                  100.0 * valid / (depth->width * depth->height));
    }
  }
  return 0;
}

int RunLive(const bs::SessionReader& session, const std::string& config) {
  bs_engine* engine = bs_create(config.c_str());
  if (engine == nullptr) {
    std::fprintf(stderr, "engine creation failed\n");
    return 1;
  }

  if (bs_live_begin(engine, session.dir().c_str()) != BS_OK) {
    std::fprintf(stderr, "live_begin failed: %s\n", bs_last_error(engine));
    bs_destroy(engine);
    return 1;
  }

  uint32_t fed = 0;
  for (const uint32_t frame_id : session.frame_ids()) {
    const auto meta = session.ReadMeta(frame_id);
    const auto depth = session.ReadDepth(frame_id);
    const auto jpeg = session.ReadImageBytes(frame_id);
    if (!meta || !depth || !jpeg) {
      std::fprintf(stderr, "frame %06u unreadable; skipping\n", frame_id);
      continue;
    }
    const cv::Mat img =
        cv::imdecode(*jpeg, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      std::fprintf(stderr, "frame %06u jpeg decode failed; skipping\n", frame_id);
      continue;
    }
    const std::vector<float> depth_f32 = depth->ToFloat();

    bs_frame_in frame{};
    frame.frame_id = frame_id;
    frame.t_capture = meta->t_capture;
    frame.t_depth = meta->t_depth;
    frame.luma = img.data;
    frame.luma_width = img.cols;
    frame.luma_height = img.rows;
    frame.luma_stride = static_cast<int32_t>(img.step);
    frame.depth = depth_f32.data();
    frame.depth_width = depth->width;
    frame.depth_height = depth->height;
    frame.fx = meta->intrinsics.fx;
    frame.fy = meta->intrinsics.fy;
    frame.cx = meta->intrinsics.cx;
    frame.cy = meta->intrinsics.cy;
    frame.dfx = meta->depth_intrinsics.fx;
    frame.dfy = meta->depth_intrinsics.fy;
    frame.dcx = meta->depth_intrinsics.cx;
    frame.dcy = meta->depth_intrinsics.cy;

    const bs_result r = bs_live_feed(engine, &frame);
    if (r == BS_OK) {
      ++fed;
    } else if (r != BS_ERR_BUSY) {
      std::fprintf(stderr, "feed(%06u) error: %s\n", frame_id,
                   bs_last_error(engine));
    }
  }

  bs_live_status status{};
  bs_live_poll_status(engine, &status);
  std::printf("live replay: fed %u frames, engine processed %u, state=%d, "
              "keyframes=%u, points=%u\n",
              fed, status.frames_processed, status.state, status.keyframes,
              status.map_points);

  const bs_result end = bs_live_end(engine);
  if (end != BS_OK) {
    std::fprintf(stderr, "live_end failed: %s\n", bs_last_error(engine));
  }
  bs_destroy(engine);
  return end == BS_OK ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc >= 2 && std::strcmp(argv[1], "--version") == 0) {
    std::printf("bs_replay %s\n", bs_version());
    return 0;
  }
  if (argc >= 2 && std::strcmp(argv[1], "--selftest") == 0) {
    char buf[512];
    const bs_result r = bs_selftest(buf, sizeof(buf));
    std::printf("%s\n", buf);
    return r == BS_OK ? 0 : 1;
  }
  if (argc < 2 || argv[1][0] == '-') {
    PrintUsage();
    return 2;
  }

  const std::string session_dir = argv[1];
  bool do_info = false;
  bool do_live = false;
  std::string config = "{}";
  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--info") do_info = true;
    else if (arg == "--live") do_live = true;
    else if (arg == "--config" && i + 1 < argc) config = argv[++i];
    else {
      std::fprintf(stderr, "unknown option: %s\n", arg.c_str());
      PrintUsage();
      return 2;
    }
  }
  if (!do_info && !do_live) do_info = true;

  auto session = bs::SessionReader::Open(session_dir);
  if (!session) {
    std::fprintf(stderr, "cannot open session: %s\n", session_dir.c_str());
    return 1;
  }

  if (do_info) {
    const int rc = RunInfo(*session);
    if (rc != 0 || !do_live) return rc;
  }
  return RunLive(*session, config);
}

// bs_synth — synthetic capture-session generator.
//
// Renders a procedurally textured room (CPU raytracer over textured planes),
// a handheld-style orbit trajectory, LiDAR-like noisy depth, and writes a
// session directory in the exact on-disk format the iPhone app produces —
// plus ground_truth/ poses that only tests read.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "bs/bs_api.h"
#include "generate.h"
#include "synth_scene.h"

namespace {

void PrintUsage() {
  std::fprintf(stderr,
               "usage: bs_synth <out_session_dir> [options]\n"
               "       bs_synth --version | --selftest\n"
               "       bs_synth --dump-plan <file.json>   capture plan +\n"
               "                    layout, for scripts/plot_capture_plan.py\n"
               "options:\n"
               "  --frames N        frame count (default 60)\n"
               "  --speed M         walking speed in m/s; derives the frame\n"
               "                    count from the path length instead of\n"
               "                    --frames (a fixed count over a fixed path\n"
               "                    is a speed, and a wrong one is unmeasurable)\n"
               "  --pan D           max rotation rate in deg/s when deriving\n"
               "                    the count (default 40)\n"
               "  --fps F           capture rate (default 30)\n"
               "  --width W         image width (default 960)\n"
               "  --height H        image height (default 720)\n"
               "  --seed S          scene/trajectory seed (default 7)\n"
               "  --no-blank-wall   texture every wall\n"
               "  --repetitive F    lay exactly-repeating tile/brick over the\n"
               "                    floor and textured walls (0..0.4). Real\n"
               "                    rooms are full of it and every other\n"
               "                    texture here is unique at every point\n"
               "  --repeat-period M tile size in metres (default 0.25)\n"
               "  --depth-noise F   depth noise scale, 0 = perfect (default 1)\n"
               "  --jpeg-quality Q  (default 88)\n"
               "  --exposure-drift F  per-frame gain swing amplitude (default 0)\n"
               "  --motion-blur     blur scaled by inter-frame motion\n"
               "  --rgb-noise F     RGB sensor-noise scale (default 1)\n"
               "  --hard            realistic capture: exposure drift + motion\n"
               "                    blur + heavier noise + coarser depth\n"
               "  --floor-calib     measure a floor plane from depth and record\n"
               "                    it in session.json, as the app does when the\n"
               "                    user points the phone at the floor first\n"
               "  --two-room        two rooms joined by a doorway, walked as a\n"
               "                    capture: circle each room, orbit every big\n"
               "                    object in it and the doorway from both\n"
               "                    sides, ending where it started\n"
               "  --scout N         prepend an N-frame scout circuit (localization\n"
               "                    scaffold only; excluded from the final solve).\n"
               "                    With --speed, N only switches it on.\n");
}

// Writes the two-room layout and the capture plan as JSON, for
// scripts/plot_capture_plan.py to draw. The geometry travels WITH the plan
// rather than being copied into the renderer: a picture of the walk drawn
// against a stale set of walls is worse than no picture, because it looks
// authoritative.
int DumpPlan(const char* path) {
  const bs::synth::TwoRoomLayout& L = bs::synth::TwoRoomLayoutSpec();
  const bs::synth::CapturePlan plan = bs::synth::BuildCapturePlan(1.5);
  FILE* f = std::fopen(path, "w");
  if (f == nullptr) {
    std::fprintf(stderr, "cannot write %s\n", path);
    return 1;
  }
  std::fprintf(f, "{\n  \"rooms\": [[%g,%g,%g,%g],[%g,%g,%g,%g]],\n", L.a.x0,
               L.a.x1, L.a.z0, L.a.z1, L.b.x0, L.b.x1, L.b.z0, L.b.z1);
  std::fprintf(f,
               "  \"divider\": {\"x0\": %g, \"x1\": %g, \"door_half\": %g, "
               "\"door_height\": %g},\n",
               L.face_a(), L.face_b(), L.door_half, L.door_height);
  std::fprintf(f, "  \"furniture\": [");
  for (size_t i = 0; i < L.furniture.size(); ++i) {
    const auto& b = L.furniture[i];
    std::fprintf(f, "%s[%g,%g,%g,%g]", i ? "," : "", b.origin.x(),
                 b.origin.x() + b.w, b.origin.z(), b.origin.z() + b.d);
  }
  // Subject distance: how far the first surface along the view ray actually
  // is. This is the number that would have caught the lap pointed square at
  // a wall — the look TARGET was a healthy 2.4 m away, and the wall it hit
  // was 0.55 m away, which no summary of the plan's own geometry could say.
  const bs::synth::Scene scene = bs::synth::MakeTwoRoomScene(7);
  std::fprintf(f, "],\n  \"plan\": [\n");
  for (size_t i = 0; i < plan.position.size(); ++i) {
    const Eigen::Vector3d dir =
        (plan.look[i] - plan.position[i]).normalized();
    const bs::synth::RayHit hit =
        bs::synth::CastRay(scene, plan.position[i], dir);
    std::fprintf(f, "%s    [%.4f,%.4f,%.4f,%.4f,%d,%.3f]",
                 i ? ",\n" : "", plan.position[i].x(), plan.position[i].z(),
                 plan.look[i].x(), plan.look[i].z(),
                 static_cast<int>(plan.phase[i]),
                 hit.plane_index < 0 ? -1.0 : hit.t);
  }
  std::fprintf(f, "\n  ]\n}\n");
  std::fclose(f);
  std::printf("wrote %zu plan samples to %s\n", plan.position.size(), path);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc >= 2 && std::strcmp(argv[1], "--version") == 0) {
    std::printf("bs_synth %s\n", bs_version());
    return 0;
  }
  if (argc >= 3 && std::strcmp(argv[1], "--dump-plan") == 0) {
    return DumpPlan(argv[2]);
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

  bs::synth::GenerateOptions options;
  options.out_dir = argv[1];
  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "missing value for %s\n", name);
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "--frames") options.frame_count = std::atoi(next("--frames"));
    else if (arg == "--speed") options.speed_mps = std::atof(next("--speed"));
    else if (arg == "--pan") options.pan_dps = std::atof(next("--pan"));
    else if (arg == "--fps") options.capture_fps = std::atof(next("--fps"));
    else if (arg == "--width") options.image_width = std::atoi(next("--width"));
    else if (arg == "--height") options.image_height = std::atoi(next("--height"));
    else if (arg == "--seed") options.seed = static_cast<uint32_t>(std::atoi(next("--seed")));
    else if (arg == "--no-blank-wall") options.blank_wall = false;
    else if (arg == "--repetitive") {
      options.repetitive = static_cast<float>(std::atof(next("--repetitive")));
    } else if (arg == "--repeat-period") {
      options.repeat_period_m = std::atof(next("--repeat-period"));
    }
    else if (arg == "--sweep") options.sweep_deg = std::atof(next("--sweep"));
    else if (arg == "--depth-noise") options.depth_noise_scale = std::atof(next("--depth-noise"));
    else if (arg == "--jpeg-quality") options.jpeg_quality = std::atoi(next("--jpeg-quality"));
    else if (arg == "--exposure-drift") options.exposure_drift = std::atof(next("--exposure-drift"));
    else if (arg == "--motion-blur") options.motion_blur = true;
    else if (arg == "--rgb-noise") options.rgb_noise_scale = std::atof(next("--rgb-noise"));
    else if (arg == "--two-room") options.two_room = true;
    else if (arg == "--floor-calib") options.floor_calibration = true;
    else if (arg == "--scout") options.scout_frames = std::atoi(next("--scout"));
    else if (arg == "--hard") {
      options.exposure_drift = 0.3;
      options.motion_blur = true;
      options.rgb_noise_scale = 2.5;
      options.depth_noise_scale = 1.5;
    }
    else {
      std::fprintf(stderr, "unknown option: %s\n", arg.c_str());
      PrintUsage();
      return 2;
    }
  }

  if (options.frame_count < 2 && options.speed_mps <= 0.0) {
    std::fprintf(stderr, "--frames must be >= 2\n");
    return 2;
  }
  if (options.capture_fps < 1.0) {
    std::fprintf(stderr, "--fps must be >= 1\n");
    return 2;
  }

  if (options.speed_mps > 0.0) {
    std::printf("generating %dx%d at %.2f m/s into %s ...\n",
                options.image_width, options.image_height, options.speed_mps,
                options.out_dir.c_str());
  } else {
    std::printf("generating %d frames (%dx%d) into %s ...\n",
                options.frame_count, options.image_width, options.image_height,
                options.out_dir.c_str());
  }
  if (!bs::synth::GenerateSession(options)) {
    std::fprintf(stderr, "generation FAILED\n");
    return 1;
  }
  std::printf("done\n");
  return 0;
}

#pragma once

#include <cstdint>
#include <string>

namespace bs::synth {

struct GenerateOptions {
  std::string out_dir;
  int frame_count = 60;
  int image_width = 960;
  int image_height = 720;
  int depth_width = 320;
  int depth_height = 240;
  double hfov_deg = 70.0;
  uint32_t seed = 7;
  bool blank_wall = true;
  double depth_noise_scale = 1.0;  // 0 disables depth noise/dropouts
  double sweep_deg = 140.0;        // total orbit arc over the sequence
  int keyframe_every = 5;
  int jpeg_quality = 88;
};

// Writes a complete synthetic RAW session (frames/, session.json,
// calibration.json) plus ground_truth/poses.json. Returns false on any IO
// failure. Deterministic for a given options struct.
bool GenerateSession(const GenerateOptions& options);

}  // namespace bs::synth

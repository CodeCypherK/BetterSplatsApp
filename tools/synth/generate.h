#pragma once

#include <cstdint>
#include <string>

namespace bs::synth {

struct GenerateOptions {
  std::string out_dir;
  int frame_count = 60;

  // --- capture rate ---
  // Every trajectory covers a fixed physical path, so a frame count IS a
  // speed: 110 frames over the 33 m two-room loop is 30 cm and 10 deg
  // between frames — 9 m/s at 30 fps, which is a sprint, not a capture. The
  // engine's own motion gate rejects poses that fast, so such a session
  // measures nothing about the tracker. Set `speed_mps` (and optionally
  // `pan_dps`) and let the frame count follow from the path; a count given
  // directly is still honored, with the implied motion reported and flagged
  // when it leaves hand-held range.
  double capture_fps = 30.0;
  double speed_mps = 0.0;   // >0 derives frame_count from the path length
  double pan_dps = 40.0;    // ceiling on rotation rate when deriving counts
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

  // --- real-world realism (all default to the original clean render) ---
  double exposure_drift = 0.0;   // per-frame gain swing amplitude (0 = off)
  bool motion_blur = false;      // blur scaled by inter-frame image motion
  double rgb_noise_scale = 1.0;  // multiplies base RGB sensor noise

  // Two connected rooms walked as a closed loop instead of a single-room
  // orbit: exercises region clustering, inter-room drift and loop closure.
  bool two_room = false;

  // Prepend an opening scout circuit of this many frames (0 = none). Those
  // frames are tagged pass="scout": a localization scaffold only, excluded
  // from the final reconstruction. Requires two_room (it walks both rooms).
  int scout_frames = 0;
};

// Writes a complete synthetic RAW session (frames/, session.json,
// calibration.json) plus ground_truth/poses.json. Returns false on any IO
// failure. Deterministic for a given options struct.
bool GenerateSession(const GenerateOptions& options);

}  // namespace bs::synth

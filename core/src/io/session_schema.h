#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace bs {

// In-memory mirror of the RAW-layer JSON documents (docs/FORMATS.md is
// normative, schema_version 1). Serialization is lossless for known fields;
// unknown fields in files are ignored on read (forward compatibility).

constexpr int kSchemaVersion = 1;

struct PinholeIntrinsics {
  double fx = 0, fy = 0, cx = 0, cy = 0;
  int ref_w = 0, ref_h = 0;
};

struct ExposureInfo {
  double duration_s = 0;
  double iso = 0;
  double bias_ev = 0;
};

struct QualityInfo {
  double lap_var = 0;
  double overexp_frac = 0;
};

struct FrameMeta {
  int schema_version = kSchemaVersion;
  uint32_t frame_id = 0;
  double t_capture = 0;
  double t_depth = 0;
  PinholeIntrinsics intrinsics;
  PinholeIntrinsics depth_intrinsics;
  std::string distortion_ref = "session";
  ExposureInfo exposure;
  QualityInfo quality;
  bool is_keyframe = false;
  std::string store_reason = "gate";  // "gate" | "kf" | "burst"
  // Which capture pass produced this frame. "scout" frames come from the
  // optional opening circuit — a fast walk of the whole space, back to the
  // walls, scanning inward. They exist to hold position later, never to be
  // reconstructed from: too fast, too far from surfaces, too few per room.
  // The final solve skips them. Absent in older sessions -> "capture".
  std::string pass = "capture";  // "capture" | "scout"

  bool is_scout() const { return pass == "scout"; }

  std::string ToJson() const;
  static std::optional<FrameMeta> FromJson(const std::string& text);
};

struct DistortionLut {
  std::vector<float> magnification;
  std::vector<float> inverse;
  double center_x = 0, center_y = 0;

  bool empty() const { return magnification.empty(); }
};

struct ColmapCameraModel {
  std::string model = "OPENCV";  // or "PINHOLE"
  std::vector<double> params;
  double fit_residual_px_max = 0;
};

struct CalibrationInfo {
  int schema_version = kSchemaVersion;
  int ref_width = 0, ref_height = 0;
  PinholeIntrinsics intrinsics_session;
  DistortionLut distortion_lut;
  std::optional<ColmapCameraModel> colmap_model;

  std::string ToJson() const;
  static std::optional<CalibrationInfo> FromJson(const std::string& text);
};

struct RegionEntry {
  uint32_t id = 0;
  std::string name;
  bool renamed = false;
};

struct SessionInfo {
  int schema_version = kSchemaVersion;
  std::string session_id;
  std::string created_utc;
  std::string end_utc;
  std::string device_model;
  std::string device_ios;
  int video_w = 0, video_h = 0, video_fps = 0;
  std::string video_pixel_format;
  int depth_w = 0, depth_h = 0;
  std::string depth_format;
  bool depth_filtering = false;
  bool af_locked = true;
  // Photometric locks. Absent in schema-v1 sessions written before the app
  // locked them, so they read back false rather than claiming a lock that
  // never happened.
  bool ae_locked = false;
  bool awb_locked = false;
  bool gdc_disabled = true;
  std::string stabilization = "off";
  uint32_t frame_count = 0;
  std::vector<uint32_t> keyframe_ids;
  std::vector<RegionEntry> regions;
  std::string app_version;

  std::string ToJson() const;
  static std::optional<SessionInfo> FromJson(const std::string& text);
};

// Ground truth written by bs_synth next to RAW (tests only, never on device).
struct GroundTruthPose {
  uint32_t frame_id = 0;
  // World-to-camera (COLMAP convention), quaternion (w,x,y,z) + translation.
  double q[4] = {1, 0, 0, 0};
  double t[3] = {0, 0, 0};
};

struct GroundTruth {
  std::vector<GroundTruthPose> poses;

  std::string ToJson() const;
  static std::optional<GroundTruth> FromJson(const std::string& text);
};

// Small helpers shared by reader/writer.
std::string ReadTextFile(const std::string& path);          // "" on failure
bool WriteTextFile(const std::string& path, const std::string& content);
std::string FrameDirName(uint32_t frame_id);                // "000042"

}  // namespace bs

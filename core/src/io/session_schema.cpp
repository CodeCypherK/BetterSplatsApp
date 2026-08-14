#include "io/session_schema.h"

#include <cstdio>
#include <fstream>
#include <sstream>

#include <nlohmann/json.hpp>

namespace bs {

using nlohmann::json;

namespace {

json ToJson(const PinholeIntrinsics& in) {
  return {{"fx", in.fx}, {"fy", in.fy}, {"cx", in.cx},
          {"cy", in.cy}, {"ref_w", in.ref_w}, {"ref_h", in.ref_h}};
}

PinholeIntrinsics IntrinsicsFromJson(const json& j) {
  PinholeIntrinsics out;
  out.fx = j.value("fx", 0.0);
  out.fy = j.value("fy", 0.0);
  out.cx = j.value("cx", 0.0);
  out.cy = j.value("cy", 0.0);
  out.ref_w = j.value("ref_w", 0);
  out.ref_h = j.value("ref_h", 0);
  return out;
}

json ParseOrDiscard(const std::string& text) {
  return json::parse(text, nullptr, /*allow_exceptions=*/false);
}

}  // namespace

std::string FrameMeta::ToJson() const {
  json j;
  j["schema_version"] = schema_version;
  j["frame_id"] = frame_id;
  j["t_capture"] = t_capture;
  j["t_depth"] = t_depth;
  j["intrinsics"] = bs::ToJson(intrinsics);
  j["depth_intrinsics"] = bs::ToJson(depth_intrinsics);
  j["distortion_ref"] = distortion_ref;
  j["exposure"] = {{"duration_s", exposure.duration_s},
                   {"iso", exposure.iso},
                   {"bias_ev", exposure.bias_ev}};
  j["quality"] = {{"lap_var", quality.lap_var},
                  {"overexp_frac", quality.overexp_frac}};
  j["is_keyframe"] = is_keyframe;
  j["store_reason"] = store_reason;
  j["pass"] = pass;
  return j.dump(2);
}

std::optional<FrameMeta> FrameMeta::FromJson(const std::string& text) {
  const json j = ParseOrDiscard(text);
  if (j.is_discarded() || !j.is_object()) return std::nullopt;
  FrameMeta m;
  m.schema_version = j.value("schema_version", 0);
  if (m.schema_version != kSchemaVersion) return std::nullopt;
  m.frame_id = j.value("frame_id", 0u);
  m.t_capture = j.value("t_capture", 0.0);
  m.t_depth = j.value("t_depth", 0.0);
  if (j.contains("intrinsics")) m.intrinsics = IntrinsicsFromJson(j["intrinsics"]);
  if (j.contains("depth_intrinsics")) {
    m.depth_intrinsics = IntrinsicsFromJson(j["depth_intrinsics"]);
  }
  m.distortion_ref = j.value("distortion_ref", "session");
  if (j.contains("exposure")) {
    const auto& e = j["exposure"];
    m.exposure.duration_s = e.value("duration_s", 0.0);
    m.exposure.iso = e.value("iso", 0.0);
    m.exposure.bias_ev = e.value("bias_ev", 0.0);
  }
  if (j.contains("quality")) {
    const auto& q = j["quality"];
    m.quality.lap_var = q.value("lap_var", 0.0);
    m.quality.overexp_frac = q.value("overexp_frac", 0.0);
  }
  m.is_keyframe = j.value("is_keyframe", false);
  m.store_reason = j.value("store_reason", "gate");
  m.pass = j.value("pass", "capture");
  return m;
}

std::string CalibrationInfo::ToJson() const {
  json j;
  j["schema_version"] = schema_version;
  j["reference"] = {{"width", ref_width}, {"height", ref_height}};
  j["intrinsics_session"] = bs::ToJson(intrinsics_session);
  j["distortion_lut"] = {{"magnification", distortion_lut.magnification},
                         {"inverse", distortion_lut.inverse},
                         {"center", {distortion_lut.center_x, distortion_lut.center_y}}};
  if (colmap_model) {
    j["colmap_model"] = {{"model", colmap_model->model},
                         {"params", colmap_model->params},
                         {"fit_residual_px_max", colmap_model->fit_residual_px_max}};
  }
  return j.dump(2);
}

std::optional<CalibrationInfo> CalibrationInfo::FromJson(const std::string& text) {
  const json j = ParseOrDiscard(text);
  if (j.is_discarded() || !j.is_object()) return std::nullopt;
  CalibrationInfo c;
  c.schema_version = j.value("schema_version", 0);
  if (c.schema_version != kSchemaVersion) return std::nullopt;
  if (j.contains("reference")) {
    c.ref_width = j["reference"].value("width", 0);
    c.ref_height = j["reference"].value("height", 0);
  }
  if (j.contains("intrinsics_session")) {
    c.intrinsics_session = IntrinsicsFromJson(j["intrinsics_session"]);
  }
  if (j.contains("distortion_lut")) {
    const auto& d = j["distortion_lut"];
    if (d.contains("magnification")) {
      c.distortion_lut.magnification = d["magnification"].get<std::vector<float>>();
    }
    if (d.contains("inverse")) {
      c.distortion_lut.inverse = d["inverse"].get<std::vector<float>>();
    }
    if (d.contains("center") && d["center"].is_array() && d["center"].size() == 2) {
      c.distortion_lut.center_x = d["center"][0].get<double>();
      c.distortion_lut.center_y = d["center"][1].get<double>();
    }
  }
  if (j.contains("colmap_model")) {
    ColmapCameraModel m;
    const auto& cm = j["colmap_model"];
    m.model = cm.value("model", "OPENCV");
    if (cm.contains("params")) m.params = cm["params"].get<std::vector<double>>();
    m.fit_residual_px_max = cm.value("fit_residual_px_max", 0.0);
    c.colmap_model = std::move(m);
  }
  return c;
}

std::string SessionInfo::ToJson() const {
  json j;
  j["schema_version"] = schema_version;
  j["session_id"] = session_id;
  j["created_utc"] = created_utc;
  if (!end_utc.empty()) j["end_utc"] = end_utc;
  j["device"] = {{"model", device_model}, {"ios", device_ios}};
  j["video"] = {{"w", video_w}, {"h", video_h}, {"fps", video_fps},
                {"pixel_format", video_pixel_format}};
  j["depth"] = {{"w", depth_w}, {"h", depth_h}, {"format", depth_format},
                {"filtering", depth_filtering}};
  j["capture"] = {{"af_locked", af_locked},
                  {"ae_locked", ae_locked},
                  {"awb_locked", awb_locked},
                  {"gdc_disabled", gdc_disabled},
                  {"stabilization", stabilization}};
  j["frame_count"] = frame_count;
  j["keyframe_ids"] = keyframe_ids;
  json regions_json = json::array();
  for (const auto& r : regions) {
    regions_json.push_back({{"id", r.id}, {"name", r.name}, {"renamed", r.renamed}});
  }
  j["regions"] = regions_json;

  if (floor_calibration.present) {
    j["floor_calibration"] = {
        {"frame_id", floor_calibration.frame_id},
        {"normal", {floor_calibration.normal[0], floor_calibration.normal[1],
                    floor_calibration.normal[2]}},
        {"offset_m", floor_calibration.offset_m},
        {"rmse_m", floor_calibration.rmse_m},
        {"incidence_deg", floor_calibration.incidence_deg},
        {"inliers", floor_calibration.inliers}};
  }
  j["app_version"] = app_version;
  return j.dump(2);
}

std::optional<SessionInfo> SessionInfo::FromJson(const std::string& text) {
  const json j = ParseOrDiscard(text);
  if (j.is_discarded() || !j.is_object()) return std::nullopt;
  SessionInfo s;
  s.schema_version = j.value("schema_version", 0);
  if (s.schema_version != kSchemaVersion) return std::nullopt;
  s.session_id = j.value("session_id", "");
  s.created_utc = j.value("created_utc", "");
  s.end_utc = j.value("end_utc", "");
  if (j.contains("device")) {
    s.device_model = j["device"].value("model", "");
    s.device_ios = j["device"].value("ios", "");
  }
  if (j.contains("video")) {
    const auto& v = j["video"];
    s.video_w = v.value("w", 0);
    s.video_h = v.value("h", 0);
    s.video_fps = v.value("fps", 0);
    s.video_pixel_format = v.value("pixel_format", "");
  }
  if (j.contains("depth")) {
    const auto& d = j["depth"];
    s.depth_w = d.value("w", 0);
    s.depth_h = d.value("h", 0);
    s.depth_format = d.value("format", "");
    s.depth_filtering = d.value("filtering", false);
  }
  if (j.contains("capture")) {
    const auto& c = j["capture"];
    s.af_locked = c.value("af_locked", true);
    s.ae_locked = c.value("ae_locked", false);
    s.awb_locked = c.value("awb_locked", false);
    s.gdc_disabled = c.value("gdc_disabled", true);
    s.stabilization = c.value("stabilization", "off");
  }
  s.frame_count = j.value("frame_count", 0u);
  if (j.contains("keyframe_ids")) {
    s.keyframe_ids = j["keyframe_ids"].get<std::vector<uint32_t>>();
  }
  if (j.contains("floor_calibration")) {
    const auto& f = j["floor_calibration"];
    SurfaceCalibration cal;
    cal.frame_id = f.value("frame_id", 0u);
    if (f.contains("normal") && f["normal"].size() == 3) {
      for (int i = 0; i < 3; ++i) cal.normal[i] = f["normal"][i].get<double>();
    }
    cal.offset_m = f.value("offset_m", 0.0);
    cal.rmse_m = f.value("rmse_m", 0.0);
    cal.incidence_deg = f.value("incidence_deg", 0.0);
    cal.inliers = f.value("inliers", 0);
    // A calibration is only usable if it names a frame and a real surface.
    cal.present = cal.frame_id != 0 && cal.offset_m > 0.0;
    s.floor_calibration = cal;
  }
  if (j.contains("regions")) {
    for (const auto& r : j["regions"]) {
      RegionEntry e;
      e.id = r.value("id", 0u);
      e.name = r.value("name", "");
      e.renamed = r.value("renamed", false);
      s.regions.push_back(std::move(e));
    }
  }
  s.app_version = j.value("app_version", "");
  return s;
}

std::string GroundTruth::ToJson() const {
  json arr = json::array();
  for (const auto& p : poses) {
    arr.push_back({{"frame_id", p.frame_id},
                   {"q", {p.q[0], p.q[1], p.q[2], p.q[3]}},
                   {"t", {p.t[0], p.t[1], p.t[2]}}});
  }
  json j;
  j["schema_version"] = kSchemaVersion;
  j["convention"] = "world_to_cam";
  j["poses"] = arr;
  return j.dump(2);
}

std::optional<GroundTruth> GroundTruth::FromJson(const std::string& text) {
  const json j = ParseOrDiscard(text);
  if (j.is_discarded() || !j.is_object()) return std::nullopt;
  if (j.value("schema_version", 0) != kSchemaVersion) return std::nullopt;
  GroundTruth gt;
  for (const auto& p : j.value("poses", json::array())) {
    GroundTruthPose pose;
    pose.frame_id = p.value("frame_id", 0u);
    const auto q = p.value("q", std::vector<double>{1, 0, 0, 0});
    const auto t = p.value("t", std::vector<double>{0, 0, 0});
    if (q.size() != 4 || t.size() != 3) return std::nullopt;
    std::copy(q.begin(), q.end(), pose.q);
    std::copy(t.begin(), t.end(), pose.t);
    gt.poses.push_back(pose);
  }
  return gt;
}

std::string ReadTextFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return "";
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

bool WriteTextFile(const std::string& path, const std::string& content) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) return false;
  out << content;
  return static_cast<bool>(out);
}

std::string FrameDirName(uint32_t frame_id) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%06u", frame_id);
  return buf;
}

}  // namespace bs

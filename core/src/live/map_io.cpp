#include "live/map_io.h"

#include <cstdint>
#include <fstream>
#include <unordered_map>
#include <vector>

namespace bs {

namespace {

constexpr uint32_t kMagic = 0x42534D50;  // "BSMP"
constexpr uint32_t kVersion = 2;

template <typename T>
void Put(std::ofstream& out, const T& v) {
  out.write(reinterpret_cast<const char*>(&v), sizeof(T));
}

template <typename T>
bool Get(std::ifstream& in, T& v) {
  in.read(reinterpret_cast<char*>(&v), sizeof(T));
  return static_cast<bool>(in);
}

void PutMat(std::ofstream& out, const cv::Mat& m) {
  const int32_t rows = m.rows, cols = m.cols, type = m.type();
  Put(out, rows);
  Put(out, cols);
  Put(out, type);
  // Row by row: a Mat may be a view into a larger buffer, so its rows are
  // not necessarily contiguous with each other.
  for (int r = 0; r < rows; ++r) {
    out.write(reinterpret_cast<const char*>(m.ptr(r)),
              static_cast<std::streamsize>(cols * m.elemSize()));
  }
}

bool GetMat(std::ifstream& in, cv::Mat& m) {
  int32_t rows = 0, cols = 0, type = 0;
  if (!Get(in, rows) || !Get(in, cols) || !Get(in, type)) return false;
  if (rows < 0 || cols < 0 || rows > 1000000 || cols > 100000) return false;
  if (rows == 0 || cols == 0) {
    m.release();
    return true;
  }
  m.create(rows, cols, type);
  for (int r = 0; r < rows; ++r) {
    in.read(reinterpret_cast<char*>(m.ptr(r)),
            static_cast<std::streamsize>(cols * m.elemSize()));
    if (!in) return false;
  }
  return true;
}

}  // namespace

bool WriteLiveMap(const LiveMap& map, const std::string& path,
                  bool scale_locked) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) return false;

  Put(out, kMagic);
  Put(out, kVersion);
  Put(out, static_cast<uint8_t>(scale_locked ? 1 : 0));
  Put(out, static_cast<uint32_t>(map.keyframes().size()));
  Put(out, static_cast<uint32_t>(map.points().size()));

  for (const auto& kf : map.keyframes()) {
    Put(out, kf.kf_id);
    Put(out, kf.frame_id);
    Put(out, kf.t_capture);
    const double q[4] = {kf.pose.q.w(), kf.pose.q.x(), kf.pose.q.y(),
                         kf.pose.q.z()};
    for (const double v : q) Put(out, v);
    for (int i = 0; i < 3; ++i) Put(out, kf.pose.t[i]);
    Put(out, kf.K.fx);
    Put(out, kf.K.fy);
    Put(out, kf.K.cx);
    Put(out, kf.K.cy);
    Put(out, kf.K.width);
    Put(out, kf.K.height);

    Put(out, static_cast<uint32_t>(kf.features.keypoints.size()));
    for (const auto& kp : kf.features.keypoints) {
      Put(out, kp.pt.x);
      Put(out, kp.pt.y);
      Put(out, kp.size);
      Put(out, kp.angle);
      Put(out, kp.response);
      Put(out, kp.octave);
    }
    Put(out, static_cast<int32_t>(kf.features.type));
    PutMat(out, kf.features.descriptors);

    Put(out, static_cast<uint32_t>(kf.undistorted.size()));
    for (const auto& p : kf.undistorted) {
      Put(out, p.x);
      Put(out, p.y);
    }
    Put(out, static_cast<uint32_t>(kf.point_ids.size()));
    for (const int32_t id : kf.point_ids) Put(out, id);
  }

  for (const auto& [id, mp] : map.points()) {
    Put(out, mp.id);
    for (int i = 0; i < 3; ++i) Put(out, mp.X[i]);
    PutMat(out, mp.descriptor);
    Put(out, mp.mean_gradient);
    Put(out, mp.max_tri_angle_deg);
    Put(out, mp.last_reproj_err_px);
    for (int i = 0; i < 3; ++i) Put(out, mp.rgb[i]);
    Put(out, static_cast<uint32_t>(mp.observations.size()));
    for (const auto& [kf_id, feat] : mp.observations) {
      Put(out, kf_id);
      Put(out, static_cast<int32_t>(feat));
    }
  }
  return static_cast<bool>(out);
}

bool ReadLiveMap(const std::string& path, LiveMap& map,
                 bool* scale_locked) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;

  uint32_t magic = 0, version = 0, kf_count = 0, point_count = 0;
  if (!Get(in, magic) || magic != kMagic) return false;
  if (!Get(in, version) || version != kVersion) return false;
  uint8_t locked = 0;
  if (!Get(in, locked)) return false;
  if (scale_locked != nullptr) *scale_locked = locked != 0;
  if (!Get(in, kf_count) || !Get(in, point_count)) return false;
  if (kf_count > 100000 || point_count > 5000000) return false;

  // Ids are reassigned by LiveMap on insert, so cross-references are
  // rewritten through these maps rather than trusted from the file.
  std::unordered_map<uint32_t, uint32_t> kf_id_map;
  std::vector<std::vector<int32_t>> stored_point_ids(kf_count);

  for (uint32_t i = 0; i < kf_count; ++i) {
    Keyframe kf;
    uint32_t stored_kf_id = 0;
    if (!Get(in, stored_kf_id) || !Get(in, kf.frame_id) ||
        !Get(in, kf.t_capture)) {
      return false;
    }
    double q[4], t[3];
    for (double& v : q) {
      if (!Get(in, v)) return false;
    }
    for (double& v : t) {
      if (!Get(in, v)) return false;
    }
    kf.pose.q = Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized();
    kf.pose.t = Eigen::Vector3d(t[0], t[1], t[2]);
    if (!Get(in, kf.K.fx) || !Get(in, kf.K.fy) || !Get(in, kf.K.cx) ||
        !Get(in, kf.K.cy) || !Get(in, kf.K.width) || !Get(in, kf.K.height)) {
      return false;
    }

    uint32_t kp_count = 0;
    if (!Get(in, kp_count) || kp_count > 1000000) return false;
    kf.features.keypoints.resize(kp_count);
    for (auto& kp : kf.features.keypoints) {
      if (!Get(in, kp.pt.x) || !Get(in, kp.pt.y) || !Get(in, kp.size) ||
          !Get(in, kp.angle) || !Get(in, kp.response) || !Get(in, kp.octave)) {
        return false;
      }
    }
    int32_t feature_type = 0;
    if (!Get(in, feature_type)) return false;
    kf.features.type = static_cast<FeatureType>(feature_type);
    if (!GetMat(in, kf.features.descriptors)) return false;

    uint32_t undist_count = 0;
    if (!Get(in, undist_count) || undist_count > 1000000) return false;
    kf.undistorted.resize(undist_count);
    for (auto& p : kf.undistorted) {
      if (!Get(in, p.x) || !Get(in, p.y)) return false;
    }
    uint32_t pid_count = 0;
    if (!Get(in, pid_count) || pid_count > 1000000) return false;
    std::vector<int32_t> point_ids(pid_count);
    for (auto& id : point_ids) {
      if (!Get(in, id)) return false;
    }
    // Filled in after point ids are remapped.
    stored_point_ids[i] = point_ids;
    kf.point_ids.assign(pid_count, -1);
    kf.from_scaffold = true;

    const Keyframe& added = map.AddKeyframe(std::move(kf));
    kf_id_map[stored_kf_id] = added.kf_id;
  }

  std::unordered_map<int32_t, int32_t> point_id_map;
  for (uint32_t i = 0; i < point_count; ++i) {
    MapPoint mp;
    int32_t stored_id = 0;
    if (!Get(in, stored_id)) return false;
    double x[3];
    for (double& v : x) {
      if (!Get(in, v)) return false;
    }
    mp.X = Eigen::Vector3d(x[0], x[1], x[2]);
    if (!GetMat(in, mp.descriptor)) return false;
    if (!Get(in, mp.mean_gradient) || !Get(in, mp.max_tri_angle_deg) ||
        !Get(in, mp.last_reproj_err_px)) {
      return false;
    }
    for (int c = 0; c < 3; ++c) {
      if (!Get(in, mp.rgb[c])) return false;
    }
    uint32_t obs_count = 0;
    if (!Get(in, obs_count) || obs_count > 100000) return false;
    for (uint32_t o = 0; o < obs_count; ++o) {
      uint32_t kf_id = 0;
      int32_t feat = 0;
      if (!Get(in, kf_id) || !Get(in, feat)) return false;
      const auto it = kf_id_map.find(kf_id);
      if (it == kf_id_map.end()) continue;  // dangling reference: drop it
      mp.observations.emplace_back(it->second, feat);
    }
    mp.from_scaffold = true;
    const MapPoint& added = map.AddPoint(std::move(mp));
    point_id_map[stored_id] = added.id;
  }

  // Rewrite per-keyframe feature associations with the new point ids.
  uint32_t index = 0;
  for (auto& kf : map.keyframes()) {
    if (index >= stored_point_ids.size()) break;
    const auto& stored = stored_point_ids[index++];
    for (size_t f = 0; f < stored.size() && f < kf.point_ids.size(); ++f) {
      if (stored[f] < 0) continue;
      const auto it = point_id_map.find(stored[f]);
      kf.point_ids[f] = it == point_id_map.end() ? -1 : it->second;
    }
  }
  return true;
}

}  // namespace bs

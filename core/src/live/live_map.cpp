#include "live/live_map.h"

#include <algorithm>

#include "fusion/scale.h"

namespace bs {

double MapPoint::TextureWeightNow(double tex_floor) const {
  return TextureWeight(mean_gradient, max_tri_angle_deg, 25.0, 4.0, tex_floor);
}

Keyframe& LiveMap::AddKeyframe(Keyframe kf) {
  kf.kf_id = next_kf_id_++;
  keyframes_.push_back(std::move(kf));
  return keyframes_.back();
}

MapPoint& LiveMap::AddPoint(MapPoint point) {
  point.id = next_point_id_++;
  auto [it, _] = points_.emplace(point.id, std::move(point));
  return it->second;
}

void LiveMap::RemovePoint(int32_t id) {
  auto it = points_.find(id);
  if (it == points_.end()) return;
  for (const auto& [kf_id, feat_idx] : it->second.observations) {
    if (Keyframe* kf = FindKeyframe(kf_id)) {
      if (feat_idx >= 0 && feat_idx < static_cast<int>(kf->point_ids.size()) &&
          kf->point_ids[feat_idx] == id) {
        kf->point_ids[feat_idx] = -1;
      }
    }
  }
  points_.erase(it);
}

Keyframe* LiveMap::FindKeyframe(uint32_t kf_id) {
  for (auto& kf : keyframes_) {
    if (kf.kf_id == kf_id) return &kf;
  }
  return nullptr;
}

const Keyframe* LiveMap::FindKeyframe(uint32_t kf_id) const {
  for (const auto& kf : keyframes_) {
    if (kf.kf_id == kf_id) return &kf;
  }
  return nullptr;
}

MapPoint* LiveMap::FindPoint(int32_t id) {
  auto it = points_.find(id);
  return it == points_.end() ? nullptr : &it->second;
}

std::vector<std::pair<uint32_t, int>> LiveMap::Covisible(uint32_t kf_id,
                                                        int min_shared) const {
  const Keyframe* kf = FindKeyframe(kf_id);
  if (kf == nullptr) return {};

  std::map<uint32_t, int> shared;
  for (const int32_t pid : kf->point_ids) {
    if (pid < 0) continue;
    const auto it = points_.find(pid);
    if (it == points_.end()) continue;
    for (const auto& [other_kf, _] : it->second.observations) {
      if (other_kf != kf_id) ++shared[other_kf];
    }
  }

  std::vector<std::pair<uint32_t, int>> result;
  for (const auto& [id, count] : shared) {
    if (count >= min_shared) result.emplace_back(id, count);
  }
  std::sort(result.begin(), result.end(),
            [](const auto& a, const auto& b) { return a.second > b.second; });
  return result;
}

void LiveMap::ApplyScale(double scale) {
  for (auto& kf : keyframes_) kf.pose.t *= scale;
  for (auto& [_, point] : points_) point.X *= scale;
}

}  // namespace bs

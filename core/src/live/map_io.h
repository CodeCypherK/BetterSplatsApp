#pragma once

#include <string>

#include "live/live_map.h"

namespace bs {

// Persisting the live map so a later pass can hold position against it.
//
// The scout circuit's map is the only live product that outlives its pass:
// everything else in live/ is disposable because it can be recomputed from
// RAW, but a scaffold is only useful if the *next* pass can load it before
// it has reconstructed anything of its own. Written to live/map.bin.
//
// Depth frames and solver lookups are deliberately not persisted — they are
// per-frame products rebuilt on demand, and a scaffold is used for
// relocalization (descriptors and 3D points), not for depth fusion.

// Serializes keyframes (poses, intrinsics, keypoints, descriptors, feature
// associations) and map points (position, descriptor, observations, stats).
// `scale_locked` records whether the map is metric (LiDAR fixed its gauge).
// A scaffold that never locked scale cannot hand a metric gauge to the next
// pass, and claiming otherwise would silently mis-scale everything built on
// top of it.
bool WriteLiveMap(const LiveMap& map, const std::string& path,
                  bool scale_locked);

// Loads into `map`, which must be empty. Keyframe and point ids are
// reassigned by the map itself and cross-references are remapped, so the
// loaded scaffold obeys the same invariants as one built by tracking.
// Every loaded keyframe and point is marked `from_scaffold`.
bool ReadLiveMap(const std::string& path, LiveMap& map,
                 bool* scale_locked = nullptr);

}  // namespace bs

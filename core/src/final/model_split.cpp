#include "final/model_split.h"

#include <algorithm>
#include <map>
#include <set>
#include <unordered_map>

#include "common/log.h"

namespace bs {

namespace {

struct UnionFind {
  std::unordered_map<uint32_t, uint32_t> parent;
  std::unordered_map<uint32_t, int> size;

  uint32_t Find(uint32_t a) {
    auto it = parent.find(a);
    if (it == parent.end()) {
      parent[a] = a;
      size[a] = 1;
      return a;
    }
    if (it->second == a) return a;
    const uint32_t root = Find(it->second);
    it->second = root;
    return root;
  }
  int Size(uint32_t a) { return size[Find(a)]; }
  void Union(uint32_t a, uint32_t b) {
    const uint32_t ra = Find(a), rb = Find(b);
    if (ra == rb) return;
    parent[ra] = rb;
    size[rb] += size[ra];
  }
};

uint64_t PairKey(uint32_t a, uint32_t b) {
  if (a > b) std::swap(a, b);
  return (static_cast<uint64_t>(a) << 32) | b;
}

}  // namespace

std::vector<ModelPart> SplitModelByCovisibility(const ColmapModel& model,
                                                const SplitOptions& options) {
  std::vector<ModelPart> parts;
  const int max_images = std::max(1, options.max_images);

  // Already small enough: one part, everything in it. Callers then treat the
  // split and unsplit cases identically.
  if (static_cast<int>(model.images.size()) <= max_images) {
    ModelPart whole;
    whole.index = 1;
    whole.model = model;
    whole.primary_images = static_cast<uint32_t>(model.images.size());
    parts.push_back(std::move(whole));
    return parts;
  }

  // --- covisibility weights: how many points each pair of images shares ---
  std::unordered_map<uint64_t, int> shared;
  for (const auto& point : model.points) {
    for (size_t i = 0; i < point.track.size(); ++i) {
      for (size_t j = i + 1; j < point.track.size(); ++j) {
        const uint32_t a = point.track[i].image_id;
        const uint32_t b = point.track[j].image_id;
        if (a != b) ++shared[PairKey(a, b)];
      }
    }
  }

  // --- agglomerate strongest-first, subject to the size cap ---
  //
  // Sorting the edges once and unioning in that order is the same greedy
  // choice as repeatedly merging the strongest pair, at a fraction of the
  // cost. The cap is what turns "one connected walkthrough" into parts: the
  // merge simply stops when a part is full, and the next-strongest edge that
  // still fits starts joining somewhere else. Because doorway edges are the
  // weak ones, they are the last considered and the first to be left uncut.
  struct Edge {
    int weight;
    uint32_t a, b;
  };
  std::vector<Edge> edges;
  edges.reserve(shared.size());
  for (const auto& [key, weight] : shared) {
    edges.push_back({weight, static_cast<uint32_t>(key >> 32),
                     static_cast<uint32_t>(key & 0xffffffffu)});
  }
  // Ties broken by image id so the partition is identical run to run.
  std::sort(edges.begin(), edges.end(), [](const Edge& x, const Edge& y) {
    if (x.weight != y.weight) return x.weight > y.weight;
    if (x.a != y.a) return x.a < y.a;
    return x.b < y.b;
  });

  UnionFind uf;
  for (const auto& image : model.images) uf.Find(image.image_id);
  for (const auto& e : edges) {
    if (uf.Find(e.a) == uf.Find(e.b)) continue;
    if (uf.Size(e.a) + uf.Size(e.b) > max_images) continue;
    uf.Union(e.a, e.b);
  }

  // --- absorb undersized parts into a neighbour ---
  //
  // Deliberately allowed to exceed the cap. A part of four images is not a
  // training run, and leaving it stranded would drop coverage the capture
  // paid for; a part slightly over capacity is the lesser problem. The
  // overshoot is bounded by min_images, so a caller can still size against
  // it.
  //
  // Covisibility picks the neighbour when there is one. Some images share no
  // tracks with anything — a glance down a corridor, a frame the solve
  // registered off a handful of points — and those have no covisibility
  // neighbour at all; they attach to the spatially nearest part instead.
  // They carry no tracks, but they are still a registered camera and a real
  // photograph, which is supervision a splat trainer can use.
  std::unordered_map<uint32_t, Eigen::Vector3d> centre_of;
  for (const auto& image : model.images) {
    centre_of[image.image_id] = image.pose.CameraCenter();
  }

  for (int guard = 0; guard < 64; ++guard) {
    std::map<uint32_t, int> part_size;
    std::map<uint32_t, Eigen::Vector3d> part_sum;
    for (const auto& image : model.images) {
      const uint32_t root = uf.Find(image.image_id);
      ++part_size[root];
      part_sum[root] += centre_of[image.image_id];
    }
    if (part_size.size() < 2) break;

    // Structure each part actually holds: points at least two of its own
    // images observe. A part can clear the image count and still be useless
    // — the cap once cut out 17 images sharing 3 points, which is a training
    // run with nothing to train on.
    std::map<uint32_t, int> part_points;
    for (const auto& point : model.points) {
      std::map<uint32_t, int> seen;
      for (const auto& obs : point.track) ++seen[uf.Find(obs.image_id)];
      for (const auto& [root, count] : seen) {
        if (count >= 2) ++part_points[root];
      }
    }

    // Weakest part first, so absorption converges instead of ping-ponging
    // between two runts. "Weak" is too few images OR too little structure.
    uint32_t victim = 0;
    double victim_rank = 1.0;
    for (const auto& [root, count] : part_size) {
      const double by_images =
          static_cast<double>(count) / std::max(1, options.min_images);
      const double by_points =
          static_cast<double>(part_points[root]) /
          std::max(1.0, static_cast<double>(count) *
                            std::max(1, options.min_points_per_image));
      const double rank = std::min(by_images, by_points);
      if (rank < victim_rank) {
        victim = root;
        victim_rank = rank;
      }
    }
    if (victim == 0) break;

    // Strongest covisibility link out of this part.
    int best_weight = 0;
    uint32_t best_other = 0;
    for (const auto& e : edges) {
      const uint32_t ra = uf.Find(e.a), rb = uf.Find(e.b);
      if (ra == rb) continue;
      if (ra != victim && rb != victim) continue;
      if (e.weight > best_weight) {
        best_weight = e.weight;
        best_other = (ra == victim) ? rb : ra;
      }
    }

    if (best_weight == 0) {
      // No shared structure anywhere: fall back to proximity.
      const Eigen::Vector3d victim_centre =
          part_sum[victim] / static_cast<double>(part_size[victim]);
      double best_distance = 0;
      for (const auto& [root, count] : part_size) {
        if (root == victim) continue;
        const Eigen::Vector3d other =
            part_sum[root] / static_cast<double>(count);
        const double d = (other - victim_centre).norm();
        if (best_other == 0 || d < best_distance) {
          best_distance = d;
          best_other = root;
        }
      }
    }
    if (best_other == 0) break;
    uf.Union(victim, best_other);
  }

  // --- stable part numbering, by the smallest image id each part holds ---
  std::vector<uint32_t> image_ids;
  image_ids.reserve(model.images.size());
  for (const auto& image : model.images) image_ids.push_back(image.image_id);
  std::sort(image_ids.begin(), image_ids.end());

  std::map<uint32_t, uint32_t> root_to_part;
  std::unordered_map<uint32_t, uint32_t> part_of_image;
  uint32_t next_part = 1;
  for (const uint32_t id : image_ids) {
    const uint32_t root = uf.Find(id);
    auto [it, inserted] = root_to_part.emplace(root, next_part);
    if (inserted) ++next_part;
    part_of_image[id] = it->second;
  }
  const uint32_t part_count = next_part - 1;

  // --- points owned by each part, then context images at the seams ---
  std::unordered_map<uint32_t, const ColmapImage*> image_by_id;
  for (const auto& image : model.images) image_by_id[image.image_id] = &image;

  std::vector<std::set<uint32_t>> members(part_count + 1);
  for (const uint32_t id : image_ids) members[part_of_image[id]].insert(id);

  // How many of part P's primary-observed points each outside image sees.
  std::vector<std::unordered_map<uint32_t, int>> context_votes(part_count + 1);
  for (const auto& point : model.points) {
    std::set<uint32_t> parts_seen;
    for (const auto& obs : point.track) {
      const auto it = part_of_image.find(obs.image_id);
      if (it != part_of_image.end()) parts_seen.insert(it->second);
    }
    if (parts_seen.size() < 2) continue;  // interior point, nothing to share
    for (const auto& obs : point.track) {
      const auto it = part_of_image.find(obs.image_id);
      if (it == part_of_image.end()) continue;
      for (const uint32_t p : parts_seen) {
        if (p != it->second) ++context_votes[p][obs.image_id];
      }
    }
  }
  for (uint32_t p = 1; p <= part_count; ++p) {
    for (const auto& [image_id, votes] : context_votes[p]) {
      if (votes >= options.overlap_min_shared) members[p].insert(image_id);
    }
  }

  // --- emit one COLMAP model per part, coordinates untouched ---
  for (uint32_t p = 1; p <= part_count; ++p) {
    ModelPart part;
    part.index = p;
    part.model.camera = model.camera;

    for (const uint32_t id : image_ids) {
      if (!members[p].count(id)) continue;
      part.model.images.push_back(*image_by_id[id]);
      if (part_of_image[id] == p) ++part.primary_images;
    }

    for (const auto& point : model.points) {
      ColmapPoint kept;
      kept.point3d_id = point.point3d_id;
      kept.xyz = point.xyz;  // same world frame — never re-anchored
      kept.error = point.error;
      for (int c = 0; c < 3; ++c) kept.rgb[c] = point.rgb[c];
      for (const auto& obs : point.track) {
        if (members[p].count(obs.image_id)) kept.track.push_back(obs);
      }
      // COLMAP needs a track to be a track. One observation is not one.
      if (kept.track.size() < 2) continue;
      part.model.points.push_back(std::move(kept));
    }

    BS_LOGI("split", "part %u: %zu images (%u primary), %zu points", p,
            part.model.images.size(), part.primary_images,
            part.model.points.size());
    parts.push_back(std::move(part));
  }
  return parts;
}

}  // namespace bs

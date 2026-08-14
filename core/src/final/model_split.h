#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "final/colmap_export.h"

namespace bs {

// Splitting one reconstruction into several COLMAP models that share a
// world frame.
//
// A large facility can produce more images and points than a splat trainer
// will take in one pass. Because every part keeps the *same* coordinates —
// no re-centring, no re-scaling, no per-part gauge — the splats trained from
// them line up when loaded together. That property is the entire point, and
// the test suite pins it.
//
// Parts follow the covisibility graph, which is what makes them read as
// rooms without anyone having to label a room: images inside a space see the
// same surfaces from many angles and are strongly connected, while a doorway
// is a bottleneck only a handful of tracks survive. Cutting weakest-first
// therefore cuts at the doorways. A size cap decides how far that goes, so
// the result is bounded by what the trainer can hold rather than by how the
// building happens to be laid out.

struct SplitOptions {
  // Upper bound on images a part may hold before merging stops. This is a
  // capacity number — set it from what the trainer can take.
  int max_images = 250;
  // Parts smaller than this are absorbed into their strongest neighbour: a
  // handful of images is not worth a training run of its own.
  int min_images = 25;
  // An image joins a neighbouring part as context when it observes at least
  // this many of that part's points. Overlap at the seams is wanted, not
  // tolerated: each part should cover its own side of a doorway completely,
  // and a shared frame makes duplicated coverage free to recombine. Lower
  // means more overlap.
  int overlap_min_shared = 8;
  // A part also has to carry structure, not just images. Measured on a
  // two-room walkthrough, the size cap once cut out a group of 17 images
  // holding 3 points between them — frames that see each other barely or
  // not at all, which is a training run with nothing to train on. Parts
  // below this many points per image are absorbed like undersized ones.
  int min_points_per_image = 5;
};

struct ModelPart {
  // 1-based, in discovery order over sorted image ids — stable across runs.
  uint32_t index = 1;
  ColmapModel model;
  // Images that are this part's own, as opposed to context borrowed from a
  // neighbour. Overlap means the parts' image sets are not disjoint, and a
  // reader needs to know which part owns what.
  uint32_t primary_images = 0;
};

// Partitions `model` into parts that share its coordinate frame. Returns a
// single part containing everything when the model already fits, so callers
// need no special case. Deterministic for a given model and options.
std::vector<ModelPart> SplitModelByCovisibility(const ColmapModel& model,
                                                const SplitOptions& options);

}  // namespace bs

#pragma once

#include <string>
#include <vector>

#include "readiness/patch_grid.h"

namespace bs {

// Splat readiness recomputed from the FINAL reconstruction.
//
// The readiness scores the user sees while walking come from the live map,
// which is approximate by design and disposable. They answer "is this room
// worth more of your time right now". They are the wrong thing to ship in
// the report, which answers a different question — "is the data you are
// about to spend an hour of GPU time on actually good" — and has the benefit
// of globally bundle-adjusted geometry, pruned outliers and completed tracks.
//
// The grid itself is unchanged: PatchGrid consumes a LiveMap, so the final
// solve assembles one from its own frames and tracks rather than a second
// scoring implementation existing to drift from the first.

struct RegionReport {
  uint32_t id = 0;
  std::string name;   // "Room 1..N" in discovery order
  float score = 0;
  float sub[5] = {0, 0, 0, 0, 0};
  double area_m2 = 0;
  uint32_t patch_count = 0;
  uint32_t weak_area_count = 0;
  int worst_deficiency = -1;
  // World bounds of the region's patches. What a rescan of this room would
  // have to cover, and the only place that number exists after a solve.
  double min[3] = {0, 0, 0};
  double max[3] = {0, 0, 0};
};

struct ReadinessReport {
  bool present = false;
  float overall = 0;
  float overall_sub[5] = {0, 0, 0, 0, 0};
  std::vector<RegionReport> regions;
};

// Serializes to the "readiness" member of report.json.
std::string ReadinessReportJson(const ReadinessReport& report);

}  // namespace bs

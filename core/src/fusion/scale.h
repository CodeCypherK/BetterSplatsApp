#pragma once

#include <vector>

namespace bs {

// Metric-scale gauge estimation: the live map is born at arbitrary SfM
// scale; LiDAR depth agreement fixes it ONCE (a gauge constraint — LiDAR
// never participates in camera tracking). Samples are ratios
// d_lidar / z_triangulated collected over confident associations.
struct ScaleEstimate {
  double scale = 1.0;
  int samples = 0;
  double mad_ratio = 1.0;  // median-absolute-deviation / median
  bool locked = false;
};

struct ScaleOptions {
  int min_samples = 30;
  double max_mad_ratio = 0.15;
};

// Robust median + MAD gate. `locked` only when enough mutually-consistent
// samples exist; callers keep collecting otherwise.
ScaleEstimate EstimateScale(std::vector<double> ratios,
                            const ScaleOptions& options = {});

// The adaptive fusion texture term (docs/ARCHITECTURE.md):
//   w_tex = max(floor, 1 - sat(grad / grad_full) * sat(angle / angle_full))
// Strong texture AND strong triangulation shrink LiDAR pull to the floor;
// weak evidence keeps full anchoring.
double TextureWeight(double mean_gradient, double tri_angle_deg,
                     double grad_full = 25.0, double angle_full_deg = 4.0,
                     double floor = 0.15);

}  // namespace bs

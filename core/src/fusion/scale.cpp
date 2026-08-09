#include "fusion/scale.h"

#include <algorithm>
#include <cmath>

namespace bs {

namespace {

double Median(std::vector<double>& values) {
  const size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + mid, values.end());
  return values[mid];
}

}  // namespace

ScaleEstimate EstimateScale(std::vector<double> ratios,
                            const ScaleOptions& options) {
  ScaleEstimate estimate;
  ratios.erase(std::remove_if(ratios.begin(), ratios.end(),
                              [](double r) {
                                return !std::isfinite(r) || r <= 0.0;
                              }),
               ratios.end());
  estimate.samples = static_cast<int>(ratios.size());
  if (ratios.empty()) return estimate;

  const double median = Median(ratios);
  std::vector<double> deviations;
  deviations.reserve(ratios.size());
  for (const double r : ratios) deviations.push_back(std::abs(r - median));
  const double mad = Median(deviations);

  estimate.scale = median;
  estimate.mad_ratio = median > 0 ? mad / median : 1.0;
  estimate.locked = estimate.samples >= options.min_samples &&
                    estimate.mad_ratio < options.max_mad_ratio;
  return estimate;
}

double TextureWeight(double mean_gradient, double tri_angle_deg,
                     double grad_full, double angle_full_deg, double floor) {
  const double sat_grad = std::clamp(mean_gradient / grad_full, 0.0, 1.0);
  const double sat_angle = std::clamp(tri_angle_deg / angle_full_deg, 0.0, 1.0);
  return std::max(floor, 1.0 - sat_grad * sat_angle);
}

}  // namespace bs

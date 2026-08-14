#include "final/image_report.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

namespace bs {
namespace {

// std::fixed prints a NaN as "nan" and an infinity as "inf", neither of
// which is JSON. report.json is parsed by the app, by validate_colmap.py and
// by whatever the user points at it, so one bad double would take the whole
// file down rather than one field. Nothing here is expected to produce one —
// this is a guard, not a fix for a known bug.
double Finite(double v) { return std::isfinite(v) ? v : 0.0; }

// Median of a copy. Small n, called twice; clarity beats nth_element here.
double Median(std::vector<double> values) {
  if (values.empty()) return 0;
  std::sort(values.begin(), values.end());
  const size_t mid = values.size() / 2;
  if (values.size() % 2 == 1) return values[mid];
  return 0.5 * (values[mid - 1] + values[mid]);
}

}  // namespace

ImageReport FlagImages(std::vector<ImageQuality> images,
                       const ImageReportThresholds& thresholds) {
  ImageReport out;
  out.images = std::move(images);
  if (out.images.empty()) return out;

  // The blur baseline comes from every frame that has a reading, registered
  // or not. Registration is not a sharpness filter — a frame can be perfectly
  // sharp and still fail to register because it saw a blank wall.
  std::vector<double> sharpness;
  std::vector<double> observations;
  for (const auto& image : out.images) {
    if (image.lap_var > 0) sharpness.push_back(image.lap_var);
    if (image.registered) {
      observations.push_back(static_cast<double>(image.observations));
    }
  }
  out.median_lap_var = Median(sharpness);
  out.median_observations = Median(observations);

  const double blur_floor = out.median_lap_var * thresholds.blur_frac_of_median;
  const double obs_floor =
      out.median_observations * thresholds.weak_obs_frac_of_median;

  for (auto& image : out.images) {
    image.unregistered = !image.registered;
    // A reading of 0 means the capture side recorded nothing, not that the
    // frame was perfectly smooth. Flagging it would be inventing a fact.
    image.blurry = out.median_lap_var > 0 && image.lap_var > 0 &&
                   image.lap_var < blur_floor;
    image.overexposed = image.overexp_frac > thresholds.max_overexp_frac;
    // Only meaningful for a frame that got a pose: an unregistered frame has
    // no observations by definition, and saying so twice helps nobody.
    image.weakly_observed = image.registered && obs_floor > 0 &&
                            static_cast<double>(image.observations) < obs_floor;

    out.blurry += image.blurry;
    out.overexposed += image.overexposed;
    out.weakly_observed += image.weakly_observed;
    out.unregistered += image.unregistered;
  }
  return out;
}

std::string ImageReportJson(const ImageReport& report, size_t max_listed) {
  // Flagged first, then by frame id, so a truncated list is the half worth
  // reading. Sorting a copy of the pointers keeps `report` const.
  std::vector<const ImageQuality*> ordered;
  ordered.reserve(report.images.size());
  for (const auto& image : report.images) ordered.push_back(&image);
  std::stable_sort(ordered.begin(), ordered.end(),
                   [](const ImageQuality* a, const ImageQuality* b) {
                     if (a->flagged() != b->flagged()) return a->flagged();
                     return a->frame_id < b->frame_id;
                   });

  std::ostringstream out;
  out.precision(4);
  out << std::fixed;
  out << "  \"image_flags\": {\n"
      << "    \"blurry\": " << report.blurry << ",\n"
      << "    \"overexposed\": " << report.overexposed << ",\n"
      << "    \"weakly_observed\": " << report.weakly_observed << ",\n"
      << "    \"unregistered\": " << report.unregistered << ",\n"
      << "    \"median_lap_var\": " << Finite(report.median_lap_var) << ",\n"
      << "    \"median_observations\": " << Finite(report.median_observations) << ",\n"
      << "    \"images_listed\": " << std::min(max_listed, ordered.size())
      << " },\n";

  out << "  \"images\": [\n";
  const size_t listed = std::min(max_listed, ordered.size());
  for (size_t i = 0; i < listed; ++i) {
    const ImageQuality& image = *ordered[i];
    out << "    {\"frame_id\": " << image.frame_id << ", \"name\": \""
        << image.name << "\", \"registered\": "
        << (image.registered ? "true" : "false")
        << ", \"observations\": " << image.observations
        << ", \"reproj_rmse_px\": " << Finite(image.reproj_rmse_px)
        << ", \"lap_var\": " << Finite(image.lap_var)
        << ", \"overexp_frac\": " << Finite(image.overexp_frac) << ", \"flags\": [";
    bool first = true;
    auto emit = [&](bool on, const char* name) {
      if (!on) return;
      if (!first) out << ", ";
      out << '"' << name << '"';
      first = false;
    };
    emit(image.blurry, "blurry");
    emit(image.overexposed, "overexposed");
    emit(image.weakly_observed, "weakly_observed");
    emit(image.unregistered, "unregistered");
    out << "]}" << (i + 1 < listed ? "," : "") << "\n";
  }
  out << "  ],\n";
  return out.str();
}

}  // namespace bs

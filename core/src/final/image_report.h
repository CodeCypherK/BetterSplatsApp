#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace bs {

// Per-image quality accounting for report.json: which frames are dragging
// the model down, and why.
//
// A Gaussian splat bakes appearance into the radiance field, so a handful of
// smeared or blown-out frames do visible damage out of all proportion to
// their number — and nothing in the pipeline currently tells the user which
// ones they are. The aggregate metrics say the reconstruction is fine; the
// user still gets a soft patch on one wall and no way to find out why.
//
// Every threshold here is RELATIVE to the session's own distribution.
// Laplacian variance is not comparable between scenes — a richly textured
// room reads several times higher than a white-walled one at identical
// sharpness — so an absolute "blurry below 100" flag would mark every frame
// in a plain room and none in a busy one. Overexposure is the exception: a
// clipped highlight is clipped regardless of what else is in the scene.

struct ImageQuality {
  uint32_t frame_id = 0;
  std::string name;          // filename inside images/
  bool registered = false;   // did the solve give it a pose
  uint32_t observations = 0; // surviving 2D-3D correspondences
  double reproj_rmse_px = 0;
  double lap_var = 0;        // capture-time sharpness proxy
  double overexp_frac = 0;

  // Why this frame is worth a user's attention. Empty for a healthy frame.
  bool blurry = false;
  bool overexposed = false;
  bool weakly_observed = false;
  bool unregistered = false;

  bool flagged() const {
    return blurry || overexposed || weakly_observed || unregistered;
  }
};

struct ImageReportThresholds {
  // Fraction of the session's MEDIAN sharpness below which a frame is called
  // blurry. Relative because lap_var is scene-dependent; the median rather
  // than the mean because a few very sharp frames should not drag the bar up.
  double blur_frac_of_median = 0.5;
  // Clipped highlights destroy information outright, so this one is absolute.
  double max_overexp_frac = 0.05;
  // Fraction of the session's median observation count below which a
  // registered frame is contributing little.
  double weak_obs_frac_of_median = 0.25;
};

struct ImageReport {
  std::vector<ImageQuality> images;
  uint32_t blurry = 0;
  uint32_t overexposed = 0;
  uint32_t weakly_observed = 0;
  uint32_t unregistered = 0;
  double median_lap_var = 0;
  double median_observations = 0;
};

// Fills the flags and counts on an already-populated image list. Separated
// from the solve so the relative-threshold logic can be tested directly.
ImageReport FlagImages(std::vector<ImageQuality> images,
                       const ImageReportThresholds& thresholds = {});

// Serializes to the "images" and "image_flags" members of report.json.
// `max_listed` caps the per-image array — the counts always reflect every
// image, and flagged images are listed first so a truncated list is still
// the useful half of it.
std::string ImageReportJson(const ImageReport& report, size_t max_listed = 1000);

}  // namespace bs

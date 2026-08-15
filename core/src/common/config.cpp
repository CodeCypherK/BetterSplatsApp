#include "common/config.h"

#include <nlohmann/json.hpp>

#include "common/log.h"

namespace bs {

using nlohmann::json;

namespace {

template <typename T>
void get_to(const json& j, const char* key, T& out) {
  auto it = j.find(key);
  if (it == j.end()) return;
  try {
    out = it->get<T>();
  } catch (const json::exception&) {
    BS_LOGW("config", "key '%s' has wrong type; keeping default", key);
  }
}

}  // namespace

EngineConfig EngineConfig::FromJson(const char* text, bool* ok) {
  EngineConfig c;
  if (ok) *ok = true;
  if (text == nullptr || text[0] == '\0') return c;

  json j = json::parse(text, /*cb=*/nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded() || !j.is_object()) {
    if (ok) *ok = false;
    BS_LOGW("config", "malformed config JSON; using defaults");
    return c;
  }

  get_to(j, "log_level", c.log_level);

  get_to(j, "live_orb_features", c.live_orb_features);
  get_to(j, "live_orb_levels", c.live_orb_levels);
  get_to(j, "live_orb_scale", c.live_orb_scale);
  get_to(j, "live_fast_threshold", c.live_fast_threshold);
  get_to(j, "live_fast_threshold_min", c.live_fast_threshold_min);
  get_to(j, "live_match_search_px", c.live_match_search_px);
  get_to(j, "live_match_ratio", c.live_match_ratio);
  get_to(j, "live_pnp_thresh_px", c.live_pnp_thresh_px);
  get_to(j, "live_pnp_min_inliers", c.live_pnp_min_inliers);
  get_to(j, "track_max_speed_mps", c.track_max_speed_mps);
  get_to(j, "track_max_rot_dps", c.track_max_rot_dps);
  get_to(j, "track_warn_rot_dps", c.track_warn_rot_dps);
  get_to(j, "track_max_view_cos", c.track_max_view_cos);

  get_to(j, "boot_min_matches", c.boot_min_matches);
  get_to(j, "boot_ransac_px", c.boot_ransac_px);
  get_to(j, "boot_min_cheirality", c.boot_min_cheirality);
  get_to(j, "boot_min_median_tri_deg", c.boot_min_median_tri_deg);
  get_to(j, "scale_min_samples", c.scale_min_samples);
  get_to(j, "scale_max_mad_ratio", c.scale_max_mad_ratio);

  get_to(j, "kf_min_translation_m", c.kf_min_translation_m);
  get_to(j, "kf_translation_depth_frac", c.kf_translation_depth_frac);
  get_to(j, "kf_min_rotation_deg", c.kf_min_rotation_deg);
  get_to(j, "kf_max_overlap", c.kf_max_overlap);
  get_to(j, "kf_min_tracked_inliers", c.kf_min_tracked_inliers);
  get_to(j, "kf_min_blur_lapvar", c.kf_min_blur_lapvar);
  get_to(j, "kf_max_overexposed_frac", c.kf_max_overexposed_frac);
  get_to(j, "kf_min_interval_s", c.kf_min_interval_s);
  get_to(j, "kf_force_interval_s", c.kf_force_interval_s);

  get_to(j, "store_min_translation_m", c.store_min_translation_m);
  get_to(j, "store_translation_depth_frac", c.store_translation_depth_frac);
  get_to(j, "store_min_rotation_deg", c.store_min_rotation_deg);
  get_to(j, "store_min_sharpness_frac", c.store_min_sharpness_frac);

  get_to(j, "live_max_points", c.live_max_points);

  get_to(j, "lba_window", c.lba_window);
  get_to(j, "lba_min_shared_points", c.lba_min_shared_points);
  get_to(j, "lba_max_iterations", c.lba_max_iterations);
  get_to(j, "lba_huber_px", c.lba_huber_px);
  get_to(j, "lba_max_pose_shift_m", c.lba_max_pose_shift_m);

  get_to(j, "loop_search_radius_m", c.loop_search_radius_m);
  get_to(j, "loop_exclude_recent", c.loop_exclude_recent);
  get_to(j, "loop_min_inliers", c.loop_min_inliers);

  get_to(j, "lidar_sigma_base_m", c.lidar_sigma_base_m);
  get_to(j, "lidar_sigma_quadratic", c.lidar_sigma_quadratic);
  get_to(j, "lidar_range_full_m", c.lidar_range_full_m);
  get_to(j, "lidar_range_zero_m", c.lidar_range_zero_m);
  get_to(j, "lidar_range_min_m", c.lidar_range_min_m);
  get_to(j, "lidar_max_incidence_deg", c.lidar_max_incidence_deg);
  get_to(j, "lidar_tex_floor", c.lidar_tex_floor);
  get_to(j, "lidar_gate_sigmas", c.lidar_gate_sigmas);

  get_to(j, "patch_size_m", c.patch_size_m);
  get_to(j, "scout_kf_translation_scale", c.scout_kf_translation_scale);
  get_to(j, "scout_kf_rotation_scale", c.scout_kf_rotation_scale);
  get_to(j, "readiness_weak_threshold", c.readiness_weak_threshold);

  get_to(j, "final_sift_features", c.final_sift_features);
  get_to(j, "final_orb_features", c.final_orb_features);
  get_to(j, "final_use_sift", c.final_use_sift);
  get_to(j, "final_sift_budget_mb", c.final_sift_budget_mb);
  get_to(j, "final_drop_weak_obs_frac", c.final_drop_weak_obs_frac);
  get_to(j, "final_drop_err_factor", c.final_drop_err_factor);
  get_to(j, "final_seq_window", c.final_seq_window);
  get_to(j, "final_exhaustive_below", c.final_exhaustive_below);
  get_to(j, "final_match_ratio", c.final_match_ratio);
  get_to(j, "final_ransac_px", c.final_ransac_px);
  get_to(j, "final_pair_min_inliers", c.final_pair_min_inliers);
  get_to(j, "final_tri_min_angle_deg", c.final_tri_min_angle_deg);
  get_to(j, "final_tri_max_err_px", c.final_tri_max_err_px);
  get_to(j, "final_ba_rounds", c.final_ba_rounds);
  get_to(j, "final_ba_max_iterations", c.final_ba_max_iterations);
  get_to(j, "final_ba_huber_px", c.final_ba_huber_px);
  get_to(j, "final_prune_obs_px", c.final_prune_obs_px);
  get_to(j, "final_prune_point_mean_px", c.final_prune_point_mean_px);
  get_to(j, "final_register_min_inliers", c.final_register_min_inliers);
  get_to(j, "final_register_thresh_px", c.final_register_thresh_px);
  get_to(j, "final_early_stop_frac", c.final_early_stop_frac);
  get_to(j, "final_threads", c.final_threads);
  get_to(j, "final_multi_component", c.final_multi_component);
  get_to(j, "final_component_min_frames", c.final_component_min_frames);
  get_to(j, "final_merge_min_points", c.final_merge_min_points);
  get_to(j, "final_merge_inlier_m", c.final_merge_inlier_m);
  get_to(j, "final_merge_min_inlier_frac", c.final_merge_min_inlier_frac);
  get_to(j, "final_component_grow_iters", c.final_component_grow_iters);
  get_to(j, "final_max_components", c.final_max_components);
  get_to(j, "final_include_scout", c.final_include_scout);
  get_to(j, "final_level_floor", c.final_level_floor);
  get_to(j, "final_square_walls", c.final_square_walls);
  get_to(j, "final_split_max_images", c.final_split_max_images);
  get_to(j, "final_split_min_images", c.final_split_min_images);
  get_to(j, "final_split_overlap_points", c.final_split_overlap_points);

  get_to(j, "floater_sigma_gate", c.floater_sigma_gate);
  get_to(j, "floater_min_rays", c.floater_min_rays);
  get_to(j, "floater_radius_neighbors", c.floater_radius_neighbors);
  get_to(j, "floater_radius_factor", c.floater_radius_factor);

  return c;
}

std::string EngineConfig::ToJson() const {
  json j;
  j["log_level"] = log_level;
  j["live_orb_features"] = live_orb_features;
  j["patch_size_m"] = patch_size_m;
  j["final_sift_features"] = final_sift_features;
  j["final_ba_rounds"] = final_ba_rounds;
  // Round-trips of the full set are added as the keys become load-bearing;
  // this serialization exists for the replay CLI's --dump-config.
  return j.dump(2);
}

}  // namespace bs

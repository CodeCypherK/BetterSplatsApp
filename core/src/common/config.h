#pragma once

#include <string>

namespace bs {

// Engine tuning knobs with production defaults. Parsed from the config JSON
// passed to bs_create(); unknown keys are ignored so configs stay forward-
// compatible. Every threshold that shapes reconstruction behavior belongs
// here, not inline in the algorithms, so tests and the replay CLI can sweep
// them.
struct EngineConfig {
  // --- logging ---
  int log_level = 1;  // 0 debug .. 3 error

  // --- live tracker ---
  int live_orb_features = 1200;
  int live_orb_levels = 8;
  float live_orb_scale = 1.2f;
  int live_fast_threshold = 20;
  int live_fast_threshold_min = 7;
  float live_match_search_px = 15.0f;
  float live_match_ratio = 0.8f;
  float live_pnp_thresh_px = 2.0f;
  int live_pnp_min_inliers = 15;
  int live_queue_depth = 2;

  // --- bootstrap ---
  int boot_min_matches = 80;
  float boot_ransac_px = 1.5f;
  float boot_h_over_e_max = 0.45f;   // reject rotation-dominant pairs
  float boot_min_cheirality = 0.90f;
  float boot_min_median_tri_deg = 1.2f;
  int scale_min_samples = 30;
  float scale_max_mad_ratio = 0.15f;

  // --- keyframes ---
  float kf_min_translation_m = 0.06f;
  float kf_translation_depth_frac = 0.03f;
  float kf_min_rotation_deg = 8.0f;
  float kf_max_overlap = 0.65f;
  float kf_min_blur_lapvar = 100.0f;
  float kf_max_overexposed_frac = 0.02f;
  float kf_min_interval_s = 0.15f;
  float kf_force_interval_s = 2.0f;

  // --- storage gating ---
  float store_min_translation_m = 0.05f;
  float store_min_rotation_deg = 5.0f;

  // --- live map caps (LIVE layer is disposable; caps are safe) ---
  int live_max_keyframes = 600;
  int live_max_points = 300000;

  // --- local BA ---
  int lba_window = 8;
  int lba_min_shared_points = 30;
  int lba_max_iterations = 10;
  float lba_huber_px = 2.0f;

  // --- loop closure (live) ---
  float loop_search_radius_m = 6.0f;
  int loop_exclude_recent = 20;
  int loop_min_inliers = 40;

  // --- LiDAR confidence model ---
  float lidar_sigma_base_m = 0.01f;
  float lidar_sigma_quadratic = 0.008f;  // sigma = base + q * d^2
  float lidar_range_full_m = 3.0f;
  float lidar_range_zero_m = 5.0f;
  float lidar_range_min_m = 0.25f;
  float lidar_max_incidence_deg = 60.0f;
  float lidar_tex_floor = 0.15f;   // min LiDAR pull on well-textured points
  float lidar_gate_sigmas = 3.0f;  // association gate in sigmas

  // --- readiness ---
  float patch_size_m = 0.35f;
  float readiness_weak_threshold = 70.0f;

  // --- final solve ---
  int final_sift_features = 4096;
  int final_orb_features = 3000;   // "fast" preset
  int final_seq_window = 8;
  int final_bow_top_k = 10;
  int final_exhaustive_below = 150;
  int final_max_pairs_per_image = 40;
  float final_match_ratio = 0.8f;
  float final_ransac_px = 1.25f;
  int final_pair_min_inliers = 30;
  float final_tri_min_angle_deg = 1.0f;
  float final_tri_max_err_px = 4.0f;
  int final_ba_rounds = 3;
  int final_ba_max_iterations = 40;
  float final_ba_huber_px = 2.0f;
  float final_prune_obs_px = 4.0f;
  float final_prune_point_mean_px = 2.5f;
  int final_register_min_inliers = 25;
  float final_register_thresh_px = 3.0f;
  float final_track_complete_px = 6.0f;
  float final_early_stop_frac = 0.005f;
  int final_threads = 4;

  // --- floater sweep ---
  float floater_sigma_gate = 4.0f;
  int floater_min_rays = 2;
  int floater_radius_neighbors = 8;
  float floater_radius_factor = 3.0f;

  // Parse from JSON text. Returns defaults when json is null/empty/"{}".
  // On malformed JSON returns defaults and sets *ok=false when ok!=null.
  static EngineConfig FromJson(const char* json, bool* ok = nullptr);
  std::string ToJson() const;
};

}  // namespace bs

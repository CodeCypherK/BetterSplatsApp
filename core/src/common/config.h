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
  // Motion plausibility. PnP RANSAC can converge on a geometrically
  // consistent but physically impossible pose — measured on a walkthrough, a
  // 5.9 m single-frame jump accepted with 92/114 inliers, after which every
  // local point fell outside the predicted frustum and tracking never
  // recovered. A held camera does not move faster than this, so a pose that
  // claims otherwise is rejected and relocalization takes over.
  float track_max_speed_mps = 4.0f;
  float track_max_rot_dps = 300.0f;
  // Comfortable turn rate. Above this the view sweeps into unmapped space
  // faster than keyframe pairs can triangulate it — measured on a walking
  // circuit, a 120 deg/s turn cut new points from 67 to 12 per keyframe and
  // tracking died fifteen frames later. Nothing is rejected at this rate;
  // the user is told to slow down while there is still support to hold.
  float track_warn_rot_dps = 60.0f;
  // Viewing-cone test for tracking candidates: cos of the largest angle
  // between the current viewing direction and one this point was observed
  // from (0.5 = 60 deg). ORB is not viewpoint-invariant, so points mapped
  // from far-off angles are not matchable and only crowd the search.
  double track_max_view_cos = 0.5;

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
  // Absolute floor on tracked support. kf_max_overlap is a RATIO against the
  // reference keyframe, so a keyframe with few associations keeps the ratio
  // healthy while the actual number of tracked points collapses — measured
  // on a walking sweep, support fell 53 -> 42 -> 35 and then to 1 without
  // ever tripping the ratio gate. Mapping the newly visible region before
  // support runs out is the whole job of a keyframe.
  int kf_min_tracked_inliers = 60;
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
  // A local refinement that wants to move a keyframe further than this is
  // not refining — the window was not describing the same place. Its result
  // is discarded rather than written into the map.
  float lba_max_pose_shift_m = 0.5f;

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

  // --- scout pass ---
  // The scout circuit trades frame budget for scaffold density: its frames
  // are never reconstructed, and how far apart its keyframes sit is what
  // limits a later pass's ability to relocalize into it.
  float scout_kf_translation_scale = 0.35f;
  float scout_kf_rotation_scale = 0.5f;

  // --- readiness ---
  float patch_size_m = 0.35f;
  float readiness_weak_threshold = 70.0f;

  // --- final solve ---
  int final_sift_features = 3000;
  int final_orb_features = 3000;
  // SIFT for the quality preset: 0 = never, 1 = always, 2 = auto (only
  // when the session is small enough for the transient descriptor memory).
  int final_use_sift = 2;
  int final_sift_max_frames = 250;
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

  // --- multi-component recovery ---
  // Frames the live pass never posed cannot be reached by PnP when they see
  // no already-reconstructed structure (walking into a new room). Such
  // frames are bootstrapped into their own component from image geometry,
  // scaled metrically from LiDAR, and merged back over shared tracks — so a
  // live tracking failure no longer propagates into the final result.
  bool final_multi_component = true;
  int final_component_min_frames = 4;   // smaller components aren't merged
  int final_merge_min_points = 12;      // shared 3D points needed to align
  float final_merge_inlier_m = 0.15f;   // alignment RANSAC inlier radius
  // Off by default, and deliberately: adjoining rooms genuinely share very
  // little structure, so most candidate correspondences are spurious and the
  // true overlap is a small fraction. A 25% gate rejected a correct 7%
  // alignment and made the reconstruction 8x worse (0.19 m -> 1.58 m). The
  // absolute inlier count at a tight radius is the meaningful evidence.
  float final_merge_min_inlier_frac = 0.0f;
  int final_component_grow_iters = 4;   // PnP/triangulate passes per component
  int final_max_components = 6;         // recovery attempts per solve

  // Scout-pass frames are a localization scaffold, not reconstruction input.
  // Set true only to study what they would contribute.
  bool final_include_scout = false;

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

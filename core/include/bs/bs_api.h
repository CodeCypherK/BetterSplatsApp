/*
 * bs_api.h — C ABI of the BetterSplats reconstruction engine.
 *
 * This is the only boundary between the Swift app and the C++ engine, and it
 * is also exercised verbatim by the Linux replay CLI and the test suite.
 * Rules for this header:
 *   - Pure C99: no C++ types, fixed-width integers, POD structs only.
 *   - Structs returned by polling calls contain no pointers; variable-length
 *     data (point clouds, patches) crosses via bs_snapshot_acquire/release
 *     with engine-owned buffers.
 *   - The engine never writes to the RAW layer (frames/, session.json,
 *     calibration.json); it owns live/ and final/ inside the session dir.
 */
#ifndef BS_API_H
#define BS_API_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- results */

typedef enum bs_result {
  BS_OK = 0,
  BS_ERR_INVALID_ARGUMENT = 1,
  BS_ERR_INVALID_STATE = 2,   /* call not legal in current engine state    */
  BS_ERR_IO = 3,              /* filesystem failure                        */
  BS_ERR_BAD_SESSION = 4,     /* session dir missing/corrupt/wrong schema  */
  BS_ERR_NOT_IMPLEMENTED = 5,
  BS_ERR_INTERNAL = 6,        /* details via bs_last_error()               */
  BS_ERR_BUSY = 7,            /* queue full — frame dropped (live is lossy) */
  BS_ERR_CANCELLED = 8
} bs_result;

/* ------------------------------------------------------------- lifecycle */

typedef struct bs_engine bs_engine; /* opaque */

/* config_json: engine tuning overrides, "{}" for defaults. Documented in
 * docs/ARCHITECTURE.md; unknown keys are ignored so configs stay forward-
 * compatible. Returns NULL only on allocation failure. */
bs_engine* bs_create(const char* config_json);
void bs_destroy(bs_engine* e);

/* Version of the engine, e.g. "0.1.0 (core abc1234)". Static storage. */
const char* bs_version(void);

/* Human-readable description of the most recent error on this engine.
 * Static storage owned by the engine; valid until the next failing call. */
const char* bs_last_error(const bs_engine* e);

/* Exercises Eigen, OpenCV and Ceres with tiny known-answer problems and
 * writes a short report into `buf`. Returns BS_OK when every dependency
 * produced correct results. Used by the app's diagnostics screen and by CI
 * to prove the dependency chain links and runs on every platform. */
bs_result bs_selftest(char* buf, size_t buf_len);

/* ------------------------------------------------------- depth container */

/* Encodes a float16 depth map into the BSDP container (docs/FORMATS.md),
 * LZ4-compressed when that shrinks it. Stateless; used by the Swift
 * SessionWriter so device-written files are byte-identical to the C++
 * codec. Returns NULL on invalid input; release with bs_buffer_release. */
const uint8_t* bs_depth_encode(const uint16_t* f16, int32_t width,
                               int32_t height, size_t* out_len);
void bs_buffer_release(const uint8_t* buf);

/* ------------------------------------------------------------ live input */

/* One synchronized RGB+depth observation. All buffers are borrowed for the
 * duration of the call; the engine copies what it keeps. */
typedef struct bs_frame_in {
  uint32_t frame_id;        /* caller-assigned, strictly increasing        */
  double   t_capture;       /* seconds, host clock, RGB mid-exposure       */
  double   t_depth;         /* seconds, host clock, depth timestamp        */

  /* Luma plane (full-res or downscaled by the app; engine adapts). */
  const uint8_t* luma;      /* 8-bit, row-major                            */
  int32_t  luma_width;
  int32_t  luma_height;
  int32_t  luma_stride;     /* bytes per row                               */

  /* LiDAR depth in meters. NaN/0 = invalid sample. */
  const float* depth;       /* row-major, depth_width*depth_height         */
  int32_t  depth_width;
  int32_t  depth_height;

  /* Intrinsics for the luma buffer AS DELIVERED (already scaled by app if
   * the luma is downscaled). */
  double fx, fy, cx, cy;
  /* Intrinsics of the depth map in depth-pixel units. */
  double dfx, dfy, dcx, dcy;

  /* Apple lens distortion lookup table (radial), in the reference frame of
   * `lut_ref_width` x `lut_ref_height`. May be NULL after the first frame
   * if unchanged; the engine caches the last table. */
  const float* lut;         /* magnification factors                        */
  int32_t  lut_count;
  const float* lut_inverse; /* inverse table, same count, may be NULL       */
  float    lut_center_x, lut_center_y;
  int32_t  lut_ref_width, lut_ref_height;

  /* Raw gyro rotation rate (rad/s, device frame) integrated by the app over
   * the interval since the previous fed frame. HINT ONLY: seeds the feature
   * search window during fast rotation. Never used as a pose source and
   * never exported. gyro_valid=0 when unavailable. */
  float    gyro_dx, gyro_dy, gyro_dz;
  int32_t  gyro_valid;
} bs_frame_in;

/* ------------------------------------------------------------ live output */

typedef enum bs_live_state {
  BS_LIVE_IDLE = 0,          /* before bs_live_begin                       */
  BS_LIVE_INITIALIZING = 1,  /* collecting bootstrap pair                  */
  BS_LIVE_TRACKING = 2,
  BS_LIVE_LOST = 3,          /* relocalizing                               */
  BS_LIVE_FINISHED = 4       /* after bs_live_end                          */
} bs_live_state;

typedef enum bs_guidance {
  BS_GUIDE_NONE = 0,
  BS_GUIDE_GOOD = 1,
  BS_GUIDE_MOVE_CLOSER = 2,
  BS_GUIDE_MOVE_SIDEWAYS = 3,
  BS_GUIDE_SLOW_DOWN = 4,
  BS_GUIDE_RECAPTURE = 5,
  BS_GUIDE_TRACKING_LOST = 6,
  BS_GUIDE_COVERAGE_NEEDED = 7
} bs_guidance;

typedef enum bs_store_reason {
  BS_STORE_GATE = 0,     /* passed adaptive motion/quality gate            */
  BS_STORE_KEYFRAME = 1, /* selected as live keyframe                      */
  BS_STORE_BURST = 2     /* recapture burst                                */
} bs_store_reason;

/* Directive: the app must persist frame `frame_id` from its ring buffer
 * into the RAW layer. Storage is append-only and never dropped. */
typedef struct bs_store_directive {
  uint32_t frame_id;
  int32_t  reason;      /* bs_store_reason */
  int32_t  is_keyframe; /* 1 when the live map keyframed this frame        */
} bs_store_directive;

#define BS_MAX_DIRECTIVES 32

typedef struct bs_live_status {
  int32_t state;              /* bs_live_state                             */
  uint32_t last_frame_id;     /* last frame the tracker processed          */
  uint32_t frames_fed;
  uint32_t frames_processed;  /* fed minus dropped                         */
  uint32_t keyframes;
  uint32_t map_points;

  /* Current camera pose estimate, world-to-camera (COLMAP convention):
   * x_cam = R(q) * x_world + t.  q = (w,x,y,z), valid when tracking. */
  double q[4];
  double t[3];
  int32_t pose_valid;

  /* Guidance for the status pill. params depend on the code:
   * MOVE_SIDEWAYS: dir = suggested world-space move direction, dist meters.*/
  int32_t guidance;           /* bs_guidance                               */
  float   guide_dir[3];
  float   guide_dist_m;
  uint32_t guide_region_id;

  /* Aggregate splat-readiness (0-100) across all regions, area-weighted. */
  float   readiness_overall;

  /* Tracking quality diagnostics. */
  float   inlier_ratio;       /* PnP inliers / matches, 0..1               */
  float   px_error_mean;      /* mean reprojection error of tracked points */
  float   blur_metric;        /* Laplacian variance of last luma           */
  int32_t scale_locked;       /* metric gauge locked from LiDAR agreement  */

  /* Pending storage directives (drained by this call). */
  int32_t directive_count;
  bs_store_directive directives[BS_MAX_DIRECTIVES];
} bs_live_status;

/* --------------------------------------------------------------- snapshot */

/* Flat, render-ready copies of the live (or final) reconstruction. All
 * arrays are engine-owned; hold them only between acquire and release. */

typedef struct bs_snap_point {
  float x, y, z;
  uint8_t r, g, b;
  uint8_t flags;       /* bit0: low-confidence, bit1: rejected-by-fusion   */
} bs_snap_point;

typedef struct bs_snap_camera {
  uint32_t frame_id;
  float q[4];          /* world-to-camera quaternion (w,x,y,z)             */
  float t[3];
  uint8_t is_keyframe;
} bs_snap_camera;

typedef struct bs_snap_patch {
  float cx, cy, cz;     /* patch centroid, world                           */
  float nx, ny, nz;     /* normal                                          */
  float extent;         /* patch cell size, meters                         */
  float score;          /* aggregate readiness 0-100                       */
  float sub[5];         /* geometry, pose, texture, lidar, view            */
  uint32_t region_id;
} bs_snap_patch;

typedef struct bs_snap_region {
  uint32_t region_id;
  char name[48];        /* UTF-8, NUL-terminated                           */
  float score;          /* 0-100                                           */
  float sub[5];
  float area_m2;
  uint32_t patch_count;
} bs_snap_region;

/* One ranked weak area with guidance parameters; text is generated by the
 * app from `deficiency` + the numeric params (Swift owns localization). */
typedef struct bs_snap_weak_area {
  float cx, cy, cz;
  float radius_m;
  uint32_t region_id;
  int32_t deficiency;   /* index of argmin sub-score: 0..4 as bs_snap_patch.sub */
  int32_t surface_kind; /* 0 wall, 1 floor, 2 ceiling, 3 object            */
  float move_dir[3];    /* suggested world-space direction (unit)          */
  float move_dist_m;
  float score;          /* aggregate readiness of the cluster              */
} bs_snap_weak_area;

typedef struct bs_snapshot {
  /* private handle used by release; do not touch */
  void* _h;

  const bs_snap_point*  points;
  uint32_t              point_count;
  const bs_snap_camera* cameras;
  uint32_t              camera_count;
  const bs_snap_patch*  patches;
  uint32_t              patch_count;
  const bs_snap_region* regions;
  uint32_t              region_count;
  const bs_snap_weak_area* weak_areas;
  uint32_t              weak_area_count;

  /* Axis-aligned bounds of the reconstruction, world frame. */
  float bounds_min[3], bounds_max[3];
} bs_snapshot;

/* ------------------------------------------------------------ live control */

/* session_dir: RAW session directory (may be empty at start of capture; the
 * app writes frames/ concurrently). The engine creates/owns live/ inside. */
bs_result bs_live_begin(bs_engine* e, const char* session_dir);

/* Feed one frame. Returns BS_OK when queued, BS_ERR_BUSY when the live
 * queue was full and the frame was dropped (raw storage is unaffected —
 * store directives reference only frames the tracker actually saw). */
bs_result bs_live_feed(bs_engine* e, const bs_frame_in* frame);

bs_result bs_live_poll_status(bs_engine* e, bs_live_status* out);

bs_result bs_snapshot_acquire(bs_engine* e, bs_snapshot* out);
void      bs_snapshot_release(bs_engine* e, bs_snapshot* snap);

/* Flush live state to <session>/live/ (poses.jsonl, map.bin, readiness.bin)
 * for final-solve initialization, then stop live threads. */
bs_result bs_live_end(bs_engine* e);

/* ------------------------------------------------------------ final solve */

typedef enum bs_final_stage {
  BS_STAGE_IDLE = 0,
  BS_STAGE_FEATURES = 1,
  BS_STAGE_VOCAB = 2,
  BS_STAGE_PAIRS = 3,
  BS_STAGE_MATCHING = 4,
  BS_STAGE_TRACKS = 5,
  BS_STAGE_INIT_POSES = 6,
  BS_STAGE_TRIANGULATE = 7,
  BS_STAGE_GLOBAL_BA = 8,
  BS_STAGE_FLOATER_SWEEP = 9,
  BS_STAGE_LIDAR_ALIGN = 10,
  BS_STAGE_EXPORT = 11,
  BS_STAGE_DONE = 12,
  BS_STAGE_FAILED = 13
} bs_final_stage;

typedef struct bs_final_progress {
  int32_t stage;             /* bs_final_stage                             */
  float   stage_progress;    /* 0..1 within the stage                      */
  float   total_progress;    /* 0..1 overall                               */
  int32_t running;           /* 1 while the pipeline thread is active      */
  int32_t paused_thermal;

  /* Rolling metrics for the processing screen. */
  uint32_t images_total;
  uint32_t images_registered;
  uint32_t points;
  float    reproj_rmse_px;
  float    mean_track_len;
  uint32_t ba_round;
} bs_final_progress;

/* preset: "quality" (default) or "fast". Resumes from
 * <session>/final/checkpoint.json when present. Runs on engine-owned
 * threads; poll for progress. */
bs_result bs_final_start(bs_engine* e, const char* session_dir, const char* preset);
bs_result bs_final_poll(bs_engine* e, bs_final_progress* out);
bs_result bs_final_cancel(bs_engine* e);  /* checkpoint + stop             */

/* Thermal pressure hint from the app: 0 nominal, 1 fair, 2 serious,
 * 3 critical. Engine sheds threads / pauses at >=2. */
bs_result bs_thermal_hint(bs_engine* e, int32_t level);

/* Rename a readiness region (persisted into live/ and final report). */
bs_result bs_region_rename(bs_engine* e, uint32_t region_id, const char* utf8_name);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* BS_API_H */

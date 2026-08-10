# Architecture

BetterSplats is an image-first photogrammetry capture system for LiDAR
iPhones. Camera poses come **only** from multi-view image geometry (features →
matching → robust two-view estimation → PnP → triangulation → bundle
adjustment). LiDAR is an independent depth sensor with its own confidence
model, used to validate and regularize geometry — never to track the camera.
ARKit is not used anywhere.

## Two builds, one engine

```
┌────────────────────────────┐      ┌─────────────────────────────┐
│  iPhone app (Swift/Metal)  │      │  Linux replay CLI (bs_replay)│
│  capture · UI · rendering  │      │  same sessions, same engine  │
└──────────────┬─────────────┘      └──────────────┬──────────────┘
               │  C ABI (core/include/bs/bs_api.h) │
               ▼                                   ▼
┌──────────────────────────────────────────────────────────────────┐
│                bscore — portable C++17 engine                     │
│  vision · geometry · lidar · fusion · live SfM · readiness ·      │
│  final solve · COLMAP export                                      │
│  deps: Eigen, OpenCV, Ceres                                       │
└──────────────────────────────────────────────────────────────────┘
```

The engine builds and is fully tested on Linux (this is the primary dev
loop — no Mac required) and cross-compiles to an iOS arm64 static library in
CI. `tools/replay` feeds a captured session directory through the identical
engine logic, so any on-device behavior can be reproduced and debugged on
Linux from an exported session zip.

## Three data layers

| Layer | Location | Writer | Lifetime |
|---|---|---|---|
| **RAW** | `frames/`, `session.json`, `calibration.json` | Swift `SessionWriter` (device) or `bs_synth` (tests) | **Immutable.** Never modified by anything after write. |
| **LIVE** | `live/`, in-memory live map | engine | Disposable. Approximate. Deleting it loses nothing but resume hints. |
| **FINAL** | `final/` | engine | Authoritative output. Rebuilt from RAW at any time; live poses are only an initialization. |

Two invariants the test suite enforces:

1. **Live SfM drift is never baked into raw LiDAR.** Any global LiDAR
   representation is derived on the fly from (original observation + current
   pose estimate + calibration) and is regenerable.
2. **Final LiDAR alignment uses only final optimized poses.** The fused
   LiDAR cloud (`dense.ply`) is produced after global bundle adjustment
   converges, from the untouched per-frame depth files.

## Live pipeline (capture time)

Threads inside the engine (device and replay identical):

- **Ingest** (caller thread): `bs_live_feed` copies luma + depth into a
  pooled frame, pushes to the tracker queue (bounded, drop-oldest — live is
  approximate by design; RAW storage is decided separately and never drops).
- **Tracker**: ORB features, constant-velocity motion model (raw gyro rate
  as a *search-window hint only*), guided matching against the local map,
  PnP RANSAC, pose refine. Emits keyframe decisions, storage directives and
  guidance states.
- **Mapper**: keyframe insertion, new-point triangulation, local windowed
  BA, map-point culling.
- **Loop**: visual loop detection + SE3 pose-graph correction (advisory —
  the final solve re-derives everything from matches).
- **Readiness**: patch grid + region scores + weak-area guidance at ~2 Hz.

The app polls `bs_live_poll_status` (~15 Hz) for the status pill, storage
directives and readiness strip, and `bs_snapshot_acquire` for render data.

## Final solve (after capture)

Checkpointed stage pipeline (resumable after cancel/thermal pause/app kill):
feature re-extraction → session vocabulary → pair selection (sequential +
covisibility + loops + retrieval) → verified matching → track building →
pose init from live → global triangulation → rounds of [global BA with
confidence-weighted LiDAR residuals → outlier pruning → registration of
missing frames → track completion] → floater sweep → final LiDAR alignment →
COLMAP text export + readiness report.

The gauge is metric: scale is locked once during live bootstrap from robust
LiDAR/triangulation depth agreement, and held in the final solve by the
per-observation LiDAR residuals.

## Adaptive visual/LiDAR fusion

Every LiDAR residual carries a continuous confidence weight
`w = w_edge · w_range · w_angle · w_tex`:

- `w_edge` kills fabricated depth pixels near discontinuities,
- `w_range` derates long range,
- `w_angle` derates oblique incidence,
- `w_tex` is the adaptive term: strongly-textured, well-triangulated points
  keep only a floor of LiDAR pull (fine visual detail is never flattened
  onto 320×240 depth), while low-texture/weak-parallax points (blank walls)
  get full LiDAR anchoring.

Robust losses (Huber on reprojection, Cauchy on LiDAR) plus a per-round
association gate keep either sensor from dominating through outliers.

## Splat readiness

A world-space patch grid (0.35 m cells) accumulates per-patch evidence and
scores five axes 0–100: **Geometry** (point density vs surface area,
triangulation conditioning, residuals), **Camera poses** (registered
keyframes, local-BA residuals, tracking health), **Texture** (gradient
energy, pixels-per-cm), **LiDAR coverage** (sample density × confidence),
**View overlap** (angular sector coverage, baseline/distance ratio).
Patches cluster into regions ("Room 1…") via the keyframe covisibility
graph; weak clusters generate ranked, directional guidance ("Back wall —
insufficient visual geometry. Move 1–2 ft left and capture again."). The
final report (`final/report.json`) recomputes everything from FINAL data.

## Known limitations (measured)

The synthetic harness covers a single-room orbit and, since `--two-room`, a
closed-loop walkthrough of two rooms joined by a doorway. The walkthrough
currently **fails**, and the failure is architectural rather than a tuning
problem. Recorded here so the gap is visible rather than implied by a
passing single-room test.

Measured on `bs_synth --two-room` (33 m loop, two rooms, returns to its
start), replayed at realistic frame density:

| | result | bound |
|---|---|---|
| live tracking | 17% of frames tracked (accurate when tracked: 2.3 cm) | ≥70% |
| final solve | 24–57% of frames registered (accurate when registered: 1.1 cm) | ≥90% |

Two distinct causes:

1. **The live map is single-segment.** Tracking dies facing the blank wall,
   the user walks into a room that was never mapped, and there is nothing to
   relocalize against. The system waits for relocalization instead of
   starting a new sub-map (plan risk #4, "segment-and-rejoin").
   Relocalization itself now sweeps the whole map rather than only the 20
   newest keyframes, which is necessary but not sufficient.

2. **The final solve initializes poses only from live poses** (S6), and can
   otherwise only grow the single connected component that PnP can reach.
   It has no image-based initial-pair bootstrap for frames the live system
   never posed. This means a live failure propagates into the final result —
   contrary to the design intent that the final solve recomputes everything
   and *fixes* live errors. Closing this needs multi-component incremental
   SfM: seed unposed frames from their own best two-view pair, scale each
   component metrically from LiDAR, and merge components over shared tracks.

Neither limitation affects single-room captures, which pass all bounds
clean and under `--hard` degradation. The two-room scene is therefore
available as a harness but is **not** yet a CI gate.

## Configuration

All tuning constants live in `bs::EngineConfig`
(`core/src/common/config.h`), overridable through the JSON passed to
`bs_create` — the replay CLI and tests sweep them; nothing is hard-coded in
the algorithms.

## Repository map

```
core/            C++17 engine (bscore) + gtest suite
tools/synth      synthetic session generator (ground truth for tests)
tools/replay     Linux driver: live replay + final solve on real sessions
ios/             SwiftUI app, XcodeGen project spec
scripts/         CI entry points, iOS dependency builder, IPA packaging
third_party/     vendored single-file deps (nlohmann/json, LZ4)
docs/            this file, FORMATS.md (on-disk schemas), SIDELOADING.md
```

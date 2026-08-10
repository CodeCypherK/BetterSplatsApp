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

## Multi-component recovery (S7b)

The live pass is allowed to fail. When it does — tracking dies on a blank
wall and the user walks into a room that was never mapped — those frames
see no reconstructed structure, so PnP cannot reach them and they used to
be dropped from the final model. That made a live failure propagate into
the final result, contrary to the whole point of recomputing from RAW.

S7b rebuilds them instead. While unposed frames remain, it seeds a fresh
component from the strongest verified two-view pair among them (never a
plane-dominated pair — those carry the conjugate-plane ambiguity), grows it
by PnP against its own points, anchors its scale on LiDAR exactly as the
live bootstrap does, and aligns it into the main model over the tracks the
two share. A component that shares no alignable structure is left
unregistered rather than placed on a guess.

Two properties are load-bearing, both learned by measurement:
- **The merge is rigid when the component is metric.** LiDAR is the metric
  authority; letting the fit absorb a free scale let alignment noise resize
  a whole room (9.6% global scale error).
- **The alignment RANSAC is adaptive.** Adjoining rooms share very little
  structure — a measured 6.6% inlier ratio — where a fixed 256 draws finds
  a clean sample only ~7% of the time. Iterations are recomputed from the
  best ratio seen, so weak overlaps still converge and strong ones stop
  early. For the same reason there is no inlier-*fraction* gate: a 25%
  threshold rejected a correct 6.6% alignment and made the result 8× worse.

Measured on `bs_synth --two-room` (33 m closed loop through a doorway):
registration 57% → **92%**, and geometry (RMSE after optimal alignment to
ground truth) 0.50 m → **~0.20 m**, with global scale within 1%.

## The scout pass

Instead of segmenting the live map and stitching sub-maps after the fact,
the user may walk the space once before capturing anything: a fast lap of
every room, back to the walls, camera aimed inward. That circuit is a
**localization scaffold** — its job is to make sure there is always
something to hold position against, so the "walked into a room nothing has
mapped" failure mostly cannot arise.

The geometry that makes a good scaffold is the opposite of the geometry
that makes good splat input: whole-room visibility, wide baselines, very
few frames per room, all of it walked fast and far from every surface. So
scout frames are tagged `pass="scout"` and the final solve excludes them
(docs/FORMATS.md). They still go to RAW like every other measurement —
replay needs them, and "not reconstructed from" is a solve-time decision,
never a licence to discard captured data.

Mechanically: the scout pass writes `live/map.bin` (keyframes with
descriptors, points, associations, plus whether LiDAR actually made it
metric — a scaffold that never locked scale must not hand a metric gauge to
the next pass). A capture pass loads it, starts in LOST, and relocalizes
into it, so the session keeps one world frame across passes. Scaffold
keyframes and points are held **constant** in local BA: this pass localizes
into the reference, it does not get to move it.

Two further payoffs beyond localization: the session's world frame is
established once, and the readiness room list exists before detail capture
starts — finish the circuit and the dashboard already reads "Room 1…N" with
low scores, i.e. a worklist.

## Known limitations (measured)

**Live tracking is still fragile on walking trajectories, which caps what
the scout pass delivers.** One cause has been found and fixed: PnP RANSAC
would occasionally converge on a geometrically consistent but physically
impossible pose (measured: a 5.9 m single-frame jump accepted with 92 of
114 inliers). That single bad pose is unrecoverable, because the local map
then projects outside the predicted frustum and no later frame can
re-acquire by tracking. `MotionIsPlausible` now rejects poses that exceed a
hand-held camera's speed, and relocalization takes over instead. On the
scout circuit that took tracking from frame 49 to frame 135 and doubled the
scaffold (14 → 28 keyframes, 586 → 1219 points).

What remains is a slower failure, and the measurements now point at its
cause: **new-point creation starves before the old map leaves view.** On the
scout circuit the mapper adds ~50 points per keyframe early on, but by the
last keyframe before the collapse it adds **3**, and the local BA window
shrinks with it (3741 → 1213 → 949 reprojection residuals over the final
three keyframes). Tracked support then decays (53 → 42 → 35 inliers) and
cliffs, with hundreds of stale points still projecting into view but no
longer matching.

Each of the following was measured and **ruled out**, so they are not worth
re-trying:

| hypothesis | measurement |
|---|---|
| bootstrap scale wrong | 0.329 vs 0.3293 m true — accurate to 0.1% |
| descriptors missing/invalid | zero unusable query descriptors |
| keyframe cadence too sparse | one every ~5 frames, including the frame before the failure |
| new points not created | 68 created at the last keyframe (the earlier "+3" reading was the map total, which nets against culling) |
| descriptor distance cap too tight | loosening it admits bad matches and makes things worse |
| candidate selection | viewing-cone test is neutral |
| search radius too small | widening it with predicted motion made things worse (1223 → 1191 points, scout ends lost) |
| local BA diverging | converges normally; the validation guard never fires |

What the numbers do show is that **track extensions collapse** (52 → 28 → 17
→ 9 → 7 over the final keyframes) while new points are still being created
and then culled for never being re-observed. So points are made and
immediately lost rather than never made. Note also that a diagnostic median
taken over *all* projected points is misleading here — most of them are not
actually visible (there is no occlusion reasoning), so it measures noise
rather than the tracked subset. The next attempt should instrument the
matched subset specifically, and look at why a point observed in keyframe N
fails to be re-observed in N+1.

The final solve is unaffected either way: it needs no live poses at all and
reconstructs a two-room walkthrough from images alone.

The two-room walkthrough still does not meet the single-room accuracy
bounds (≥90% registration is met; <5 cm ATE is not), so it remains a
harness rather than a CI gate. Single-room captures are unaffected and pass
every bound both clean and under `--hard` degradation.

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

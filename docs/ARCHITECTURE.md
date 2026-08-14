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

## Measuring the tracker honestly

A long investigation into "live tracking is fragile on walking
trajectories" ended with the finding that most of the fragility was in the
**harness, not the tracker** — and the way it hid is worth keeping written
down, because the same trap is easy to re-enter.

Every synthetic trajectory covers a *fixed physical path*. So a frame count
is a speed. `--frames 110` over the 33 m two-room loop is 30 cm and 10 deg
between frames: **9 m/s at 30 fps**, a sprint, and beyond the engine's own
motion-plausibility gate. Every "tracking failure" measured that way was the
tracker correctly refusing impossible motion.

Two further defects were pure geometry, both invisible in mean and 95th
percentile statistics:

- The scout circuit aimed at "the centre of whichever room you are in",
  switching rooms at the doorway — **one 174 deg frame** in an otherwise
  clean 0.7 deg/frame lap. That single frame ended tracking for the
  following 800.
- Blending the two room centres fixed the flip but put the look target *on
  top of the camera* mid-doorway, aiming the lens at the floor where the
  roll is ill-conditioned: a 4 deg/frame change of view direction came out
  as 10 deg/frame of pose.

`bs_synth` now takes `--speed` (m/s) and derives the frame count from the
measured path length, `--pan` (deg/s) binds the per-frame view slew, and
every run prints what it generated — with a NOTE when the motion is outside
hand-held range. `MeasureMotion` reports the **worst single frame**, and a
test asserts every speed-derived trajectory stays inside the engine's own
limits. The frame-count mode is still correct for final-solve harnesses:
that stage only ever sees stored frames, which really are ~30 cm apart.

Measured on the two-room scout circuit (44 m lap, 1.0 m/s, 40 deg/s pan):

| | before | after |
|---|---|---|
| scout frames tracked | 33% | **86%** |
| scout ATE (RMSE) | — | **4.3 cm**, 0.47 deg |
| scaffold | 85 kf / 3.9k pts | **224 kf / 11.8k pts** |

`bs_replay` reports per-pass motion and writes `live/poses_scout.jsonl`
separately from `live/poses.jsonl`, so the capture pass no longer erases the
evidence for the scout pass, and a replay that is feeding decimated frames
says so.

## Known limitations (measured)

**Turning outruns mapping.** A new point needs two keyframes that both see
it, so the map grows at the rate keyframes accumulate, while a turn sweeps
the leading edge of the view across unmapped space at the rotation rate.
Past roughly 100 deg/s the second wins: measured on the walking circuit, a
120 deg/s turn round a doorway took new points from 67 to 12 per keyframe
while in-view support fell 616 → 64 over fifteen frames, and tracking ended
three frames later. That rate is well inside what a wrist can do and far
below the plausibility gate, so it is not a bad pose to reject. The engine
now raises `SLOW DOWN` above `track_warn_rot_dps` (60 deg/s) while there is
still support to hold onto.

Each of the following was measured and **ruled out**, so they are not worth
re-trying:

| hypothesis | measurement |
|---|---|
| bootstrap scale wrong | 0.329 vs 0.3293 m true — accurate to 0.1% |
| descriptors missing/invalid | zero unusable query descriptors |
| keyframe cadence too sparse | one every ~5 frames, including the frame before the failure |
| new points not created | created normally (67/keyframe) until the turn rate outran them |
| descriptor distance cap too tight | loosening it admits bad matches and makes things worse |
| candidate selection | viewing-cone test is neutral |
| search radius too small | widening it with predicted motion made things worse |
| local BA diverging | converges normally; the validation guard never fires |
| relocalization candidate coverage | 8 → 64 candidates per attempt moved tracking 11.8% → 12.2% |
| scaffold unusable because ORB is viewpoint-sensitive | wrong — see below; the same scaffold now carries 62% of the capture pass |
| denser keyframes while turning | **worse**: capping the interval at 5 deg of turn took the capture pass 61.9% → 38.9%, ATE 0.075 → 0.223 m |
| baseline-aware triangulation partners | **worse**: requiring a partner ≥4% of scene depth away left tracking flat (61.9% → 61.8%) and degraded ATE 0.075 → 0.101 m, with 4% fewer points |

**Where recovery was actually failing.** For a long time a capture pass
localized into its scout scaffold on only 12% of frames, and it was tempting
to blame appearance: the scout looks *inward at room centres* while the
capture walk looks *along travel*, and ORB is not viewpoint-invariant. That
was wrong. Relocalization was succeeding — repeatedly, with 30–70 PnP
inliers — and then dying on the very next frame, over and over.

`Relocalize` set the pose but never moved `last_kf_id_`. `TrackFrame` builds
its candidate set from that keyframe and its covisibles, so every recovery
handed the following frame a local map for the place the camera had just
left. One line, and it was worth more than every parameter that had been
swept around it:

| | before | after |
|---|---|---|
| capture-pass frames tracked | 12.2% | **61.9%** |
| capture-pass ATE | 0.218 m | **0.075 m** |
| relocalization events (thrash) | 24 | **7** |
| scout frames tracked | 86.0% | **89.5%** |
| scout ends | LOST | **TRACKING** |

The lesson worth keeping: a recovery path that reports success is not
necessarily recovering. `relocalized frame N against kf K` was printed 24
times and looked like the system working.

**Overlap beats parallax in the live map.** Two attempts to buy the turning
case more triangulation baseline both made things worse, from opposite
directions: more keyframes (shorter gaps) and fewer-but-farther partners.
The first invited an explanation about short baselines producing weak
points — and then requiring *long* baselines hurt too, which kills it. What
both changes actually did was cost matches. ORB stops matching across
viewpoint change, so a distant partner returns fewer correspondences, and
the live map lives on track extensions rather than on well-conditioned
individual points; the local BA window then spans fewer, shorter tracks. So
neither more keyframes nor better-separated ones helps on its own, and the
flat interval floor plus pure-covisibility partner selection stay as they
are. Anything aimed at the turning case has to add *matches*, not geometry.

The final solve is unaffected by any of this: it needs no live poses at all
and reconstructs a two-room walkthrough from images alone.

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

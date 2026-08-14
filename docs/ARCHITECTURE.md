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

## Levelling: putting the reconstruction the right way up

Image-only structure from motion has no gravity. The world frame is anchored
on whatever the first camera happened to be doing, so a scan of an ordinary
room routinely comes out on a slope with its walls at an arbitrary bearing.
Nothing is wrong with it geometrically — every camera and point is
self-consistent — but it is unpleasant downstream and makes two scans of
adjacent spaces needlessly hard to compare.

S8b finds the floor, levels it to `y = 0` with `+Y` up, and squares the
dominant walls to the axes. It is one rigid transform applied to poses and
points together, before the floater sweep and the dense cloud, so every
later stage simply works in the levelled frame — including the split parts,
which inherit it for free. Geometry is preserved exactly; only where the
world sits changes. A unit test pins that at 1e-9 over pairwise distances.

Finding the floor is mostly a matter of ruling things out, and each rule
earned its place:

- **A wall** always buries a good part of the scene on its far side. A floor
  never does — nothing lives under a floor.
- **A ceiling** is the hard one. Orient its normal toward the cameras and it
  satisfies every test a floor does: cameras all on the positive side,
  nothing beyond it, a plausible room-height away. In a symmetric room it
  ties on inlier count, and the first implementation picked it — which
  showed up as cameras landing 0.97 m above the "floor" instead of 1.5 m.
  The two are separated by the one asymmetry a hand-held capture always
  has: the phone is carried above the mid-height of the room, so of two
  opposed extremal planes the floor is the **farther**. When the two are
  within 25 cm the scan is left alone rather than guessed at.
- **Sampling blind does not work.** Floors are low on texture and seen at a
  glancing angle, so they are a small share of tracked points and three
  random points rarely all land on one — the first run found nothing at all
  on a real reconstruction. The camera trajectory supplies the prior: a
  phone carried around a room stays at roughly one height, so the plane
  through the camera centres is horizontal and its normal is up. Only the
  axis is used, never the sign, and a trajectory too straight to define a
  plane yields no prior.

Squaring the walls is then a one-angle problem, since a levelled floor makes
walls vertical. Wall-band points project onto the ground plane as lines, and
the frame is rotated until those lines fall on the axes — scoring by the sum
of squared histogram counts, which rewards exactly that concentration.

Measured on a clean single-room solve: **cameras land at 1.494 ± 0.011 m
above the levelled floor, against a true 1.5 m.** On the two-room
walkthrough, whose geometry error is ~0.2 m, the same fit gives 1.12 ±
0.41 m — levelling is only ever as good as the reconstruction underneath it,
which is why `report.json` carries the camera height and its spread. Those
are the honest quality signals; the rotation angle is not one, since a solve
that started upside down reports 168 degrees on a perfectly good scan.

A single big plane is genuinely ambiguous — a wall two metres to your left
and a floor two metres below look identical from geometry alone — and is
documented as such rather than resolved by guessing.

## Split export: a facility a room at a time

A large facility can produce more images and points than a splat trainer
will take in one pass. `final_split_max_images` writes
`final/colmap_parts/part_NN/` **beside** the single combined model — never
instead of it — each a complete, self-validating COLMAP model with its own
`images/`, `transforms.json`, and a `parts.json` manifest.

The property that makes this worth doing is that **every part keeps the
combined model's coordinates exactly**: no re-centring, no re-scaling, no
per-part gauge. Splats trained from the parts separately load back together
already aligned, with no registration step. The test suite pins it at
1e-12 and `validate_colmap.py` re-checks it against pycolmap at 1e-9 —
measured, the delta is 0.

Parts follow the covisibility graph, which is what makes them come out as
rooms without anyone labelling a room: images inside a space see the same
surfaces from many angles and are strongly connected, while a doorway is a
bottleneck only a handful of tracks cross. Edges are sorted by shared-point
count and merged strongest-first until a part hits the size cap, so the weak
doorway edges are the last considered and the first left uncut. The cap is a
capacity number — set it from what the trainer can hold — not a claim about
the building.

Three details are load-bearing, and two of them were learned by running it:

- **Overlap at the seams is wanted, not tolerated.** An image joins a
  neighbouring part as context when it observes enough of that part's
  points, so each part covers its own side of a doorway completely. A shared
  frame makes duplicated coverage free to recombine, and the JPEGs are
  hard-linked, so it is nearly free on disk too. Measured on a two-room
  walkthrough: 100 images, 113 memberships, 13 shared.
- **Image count alone does not make a part viable.** The cap once cut out a
  group of 17 images holding 3 points between them — a training run with
  nothing to train on. Parts are now judged on structure as well as size and
  absorbed into a neighbour when either is too thin.
- **Some images share no tracks with anything.** They have no covisibility
  neighbour to be absorbed toward, so they attach to the spatially nearest
  part. They carry no tracks, but they are still a registered camera and a
  real photograph, which is supervision a trainer can use.

Absorption is deliberately allowed to exceed the cap, bounded by
`final_split_min_images`: a handful of orphaned images is not a training run
and dropping them would lose coverage the capture paid for, so a part
slightly over capacity is the lesser problem.

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

## Crossing a doorway

The scout circuit looks into the room it is **entering**, not at the doorway
it is crossing — and it is already turned by the time it gets there. A door
frame passed at arm's length sweeps through the view far too fast to match
against, and the scaffold does not need it: the doorway gets proper coverage
later, from inside-out capture and orbiting of the regions of interest
around it. What the scaffold wants at that moment is the next room's far
structure, which is distant, slow-moving, and still there seconds later.

Getting this right is a two-sided constraint, and both sides were measured:

- **Decide too late** and the turn cannot finish. Switching when the next
  room first becomes visible through the opening left the camera still
  facing backwards as it crossed the threshold, because turning is
  rate-limited.
- **Decide too early** and the switch lands back on the leg that runs along
  the far wall, where travel is *already* pointed at the doorway. The camera
  then walks straight toward what it is looking at — the degenerate case for
  parallax — and the scaffold collapses (a 5 m lead took scout tracking from
  90% to 33%).

A 3 m lead puts the decision on the approach leg, where travel runs across
the new view rather than along it.

The **path** has to cooperate, and originally it did not. The circuit cut
diagonally from room A's far corner straight to the opening and pinched to a
point at the threshold — nobody walks through a door that way, and it left
no square approach to turn on. The waypoints now trace an **H**: a
room-sized loop on each side, joined by a straight run through the opening.
The path comes down the divider wall onto the opening's axis, crosses
perpendicular to it, and picks up the next room's perimeter on the far side.
That gives the turn a full 3 m of wall to happen against, with travel
running down the wall while the view swings through 90° toward the opening —
the two across each other, which is what keeps parallax alive. The opening
itself went from 1.1 m to a 1.6 m cased gap, the sort between two living
spaces rather than an internal door.

Measured on the two-room walking session, cumulatively:

| | at the doorway | + 3 m lead | + H-shaped path |
|---|---|---|---|
| scout frames tracked | 89.6% | 89.3% | **98.6%** |
| scout ATE | 0.047 m | 0.080 m | **0.043 m** |
| capture-pass frames tracked | 61.8% | 72.5% | **73.2%** |

The scout circuit now holds tracking for essentially the whole lap.

Capture-pass pose error moved the other way over the same changes (ATE 0.10
→ 0.21 → 0.32 m, rotation 1.4° → 2.7° → 4.7°). Part of that is a selection
effect rather than a regression — ATE averages over tracked frames only, so
surviving the doorway means the hardest poses now count where before they
were simply absent — but the size of it is no longer comfortably explained
that way, and it is an open item rather than a settled caveat.

## What reaches RAW

The storage gate decides which frames are written to the immutable layer,
and therefore which frames the final solve ever sees. It was purely
geometric — store one every 5 cm or 5 deg — with no quality check at all,
while the *keyframe* gate beside it has always rejected blurred and
blown-out frames. So a smeared frame that happened to land on the 5 cm
boundary was stored, permanently, and SIFT then had to work with it.

The gate now also asks for sharpness, relative to recent typical sharpness
(the same content-adaptive threshold the keyframe gate uses, since absolute
Laplacian variance is a property of the scene as much as the optics). If the
frame is soft it waits for a better one — but only up to twice the geometric
threshold, after which it stores regardless. Coverage beats sharpness: a gap
cannot be fixed later, a soft frame can at least be down-weighted.

**This one is reasoned, not measured, and the harness is why.** Three
attempts to demonstrate it failed. Synthetic blur moves this scene's
Laplacian variance far less than its own procedural texture does, so
sharpness spread across a sequence stays around 1.2x between the 10th and
90th percentile whatever the motion — there is nothing for a selection gate
to choose between, and stored-frame sharpness lands within 0.4% of the
sequence mean with the gate on or off. Adding a footfall-and-tremor term to
the blur model (real captures do jolt at each step) did not change that
either; its only measurable effect was to push hard-scene ATE past its bound
through an amplitude that was guessed rather than measured off a device, so
it was reverted. Validating this needs a real handheld session, where blur
is intermittent and severe rather than smooth and mild.
`store_min_sharpness_frac` sets the threshold and 0 disables it.

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

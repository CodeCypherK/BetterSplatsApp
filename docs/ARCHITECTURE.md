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
graph; weak clusters generate ranked, directional guidance. The
final report (`final/report.json`) recomputes everything from FINAL data.

After the solve, `final/report.json` carries a **per-image table** —
registered, surviving observations, reprojection RMSE, capture-time
`lap_var` and `overexp_frac` — plus four flags: blurry, overexposed,
weakly_observed, unregistered. Thresholds are relative to the session's own
distribution, because `lap_var` is not comparable between scenes: a richly
textured room reads several times higher than a white-walled one at
identical sharpness, so an absolute cutoff condemns every frame in a plain
room and none in a busy one. Overexposure is the exception and stays
absolute — clipped highlights are destroyed information regardless of scene.
The app reads this back on the screen immediately before the export buttons,
which is where "train on this or rescan?" actually gets decided, and it is
far cheaper to decide there than after an hour of GPU time.

Guidance is generated as a code plus numbers and worded in Swift, never as
engine-side prose. Two rules keep it usable rather than merely correct:

**Every distance and direction is relative to the user.** The grid works in
the session's world frame, whose origin is wherever the first keyframe
landed, so a raw centroid length printed as "3.2 m away" means 3.2 m from
where the session *started* — which is not how anyone reads it. Spatial
wording goes through the live pose (`ViewerPose`), and when there is no pose
the distance is omitted rather than guessed: a number from the wrong origin
is worse than no number.

**TRACKING LOST carries a vector.** The one thing a lost user does not know
is where the map is, and it is the one thing the engine knows exactly.
`guide_dir` is the direction to the nearest keyframe in **camera**
coordinates — camera, not world, precisely because `pose_valid` is 0 when
lost, so a world vector would have nothing to resolve against. Nearest
rather than most recent: someone who walked into an unmapped corner should
be sent to the closest mapped place, which may be behind them rather than
back along the path they took. Standing on top of a keyframe and still lost
means the camera is pointed wrong, not the feet, so no arrow is emitted at
all below 0.35 m. `GuideToNearestKeyframe` is a free function so the sign
and frame conventions are unit-tested against hand-placed keyframes — an
arrow that confidently points the wrong way walks the user *away* from the
map while telling them they are going back to it.

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

Measured on `bs_synth --two-room` when that was a 33 m closed loop through a
doorway: registration 57% → **92%**, and geometry (RMSE after optimal
alignment to ground truth) 0.50 m → **~0.20 m**, with global scale within 1%.
(`--two-room` is now the 121 m circle-and-orbit capture walk below, so these
figures describe the change, not the current fixture.)

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

## The capture walk: circle, then orbit

The scout circuit answers "where am I". The capture walk answers "what does
this surface look like from every side", and that is a different shape of
path. It is three moves per room, in order:

1. **Circle the room.** One lap at the largest inset the furniture allows,
   camera facing out at the walls. This establishes the shell — walls,
   corners, ceiling line — that everything else sits inside.
2. **Orbit each large object.** All the way round, object centred in frame.
   This is what gives a surface enough distinct viewpoints to reconstruct as
   a solid rather than as a card facing whichever way the walk happened to
   go past it.
3. **Orbit the doorway** — from *both* rooms. An opening is the one place
   two captures have to agree about the same surface, and it is where a
   splat of a house shows its seam.

**Orbit distance is half the room's short dimension.** Far enough that the
object stays whole in frame with the room behind it for context, close
enough for texture. The short dimension, not the long one: half the long
side of a corridor puts you through a wall. In practice the rule is a
ceiling, not a distance — a 6×8 m room asks for 3.0 m and the walls and
furniture cut every orbit back to 1.6–2.6 m. Where the ring is blocked the
plan walks the arcs that *are* walkable and steps around the gap still
looking at the object, which is what a person does.

For step 3 to mean anything the test scene had to grow a real wall. The
divider between the two synthetic rooms used to be a zero-thickness plane
with a hole in it, so the "doorway" had no jambs and no soffit: nothing
inside the opening to see from an angle, no depth return anywhere in it, and
no reason for a capture path to orbit it. It is now a 16 cm partition with
two faces, two jambs and a soffit — the surfaces that exist only because a
wall has depth, invisible face-on and fully visible from 30 deg off-axis,
which is exactly the geometry an orbit is walked to collect.

The synthetic harness implements exactly this (`CaptureTrajectory`), because
a movement flow that is only ever described in the UI is a movement flow
nothing tests. It is computed from the layout rather than typed as
waypoints: hand-typed waypoints encode one furniture arrangement and walk
through the sofa the moment the arrangement changes. `TwoRoomWalkable()`
exposes the same floor model to tests, which assert that no pose ends up
inside a wall, the divider, or the furniture.

Measured on the two-room scene (6×8 m each), at 1.0 m/s and 40 deg/s pan:

| | value |
|---|---|
| path per room | 62 m / 57 m |
| bearings each object is framed from | 4–11 of 12 thirty-degree sectors |
| doorway bearings | 11 of 12 |
| worst single frame | 1.33 deg (exactly the declared pan limit) |
| **stored frames per room, 5 cm gate** | **944 / 867** |
| stored frames per room, 10 cm / 8 deg gate | 473 / 435 |

That last pair is the finding. The flow is right and the **gate is wrong for
it**: at the shipping `store_min_translation_m = 0.05` a single room asks for
~900 images, roughly twice the 200–500 a project is sized for. 10 cm lands in
the middle of the band, and at 2.5 m median depth 10 cm is still only a 2.3
deg baseline between neighbouring stored views. Changing it is an on-device
behaviour change with its own measurement (does final-solve registration
hold at half the density), so it is tracked in docs/BACKLOG.md rather than
done here.

Objects against a wall cannot be orbited past about 150 deg — you cannot
walk behind a sideboard — so their bearing coverage is 7–8 sectors while a
free-standing table reaches 11. That is a property of the room, not a defect
of the plan, and it is why the readiness score weights view overlap per
patch rather than per object.

## Floor calibration: measuring the floor instead of guessing it

Everything the levelling search has to reason around — floor against
ceiling, a lone ambiguous plane, a floor too sparsely tracked to find — is
answered in a couple of seconds if the user simply points the phone at the
floor before capturing. They know which surface it is, and the depth sensor
sees a dense sheet of it from a metre away: measured on the harness,
**17,000 inliers at 11 mm RMSE** against a few hundred sparse points at
23 mm from the search.

The plane is stored in that frame's **camera** coordinates, paired with its
frame id, never in world coordinates. That keeps it a measurement — an
immutable fact about what the sensor saw — which the *current* pose estimate
turns into a world plane whenever one is needed. Storing a world plane would
bake in whatever the live tracker believed at capture time and go stale the
moment the final solve moved that frame.

Two mistakes are worth recording, because both produce a confident wrong
answer rather than an error:

- **The calibration must name a frame that is really in the session.** The
  first version measured the plane from a fabricated straight-down pose and
  filed it under frame 1, whose real pose looks outward. The solve dutifully
  applied a camera-space plane against the wrong camera and put the floor
  **1.1 m out**. A calibration only means anything alongside the pose of the
  camera that took it.
- **The calibration frames need parallax.** Sweeping the phone from
  floor-ward to forward while standing still is pure rotation, so nothing
  triangulates and the solve registers none of those frames — the levelling
  then falls back to inference without anything looking wrong. Moving while
  sweeping, which is what a person does anyway, took registration from
  **71/84 to 84/84**.

On the same scene, with truth at 1.5 m: the measured floor puts the cameras
at **1.496 m** and the floor within 11 mm of `y = 0`; the inferred floor
puts them at 1.447 m. The solve prefers the calibration when its frame
registered, and says so in `report.json` as `floor_measured`; when the frame
was dropped it warns and falls back, which is verified rather than assumed.

`bs_synth --floor-calib` renders the gesture as real frames and measures the
plane from the same depth bytes it wrote to RAW, so the harness exercises
the actual path rather than asserting the answer.

### The capture step

The prompt runs over the first stored frames of an ordinary capture, not in
a mode of its own. That is deliberate: a calibration frame has to be one the
final solve registers like any other, and a separate mode would be free to
produce frames that never do. It asks the user to **point at the floor and
take a step** — the step is load-bearing rather than friction, since a
calibration captured standing still is pure rotation and the solve drops the
frame the calibration is attached to.

A reading has to hold for eight consecutive fits before it is accepted (one
good frame can be a coincidence of where the phone was pointing mid-swing),
and only ever against a frame id the session has actually written. The
verdict and its wording come from the engine — `CheckFloorPlane` and
`FloorVerdictAdvice`, both unit-tested — so the thresholds, which are claims
about how a phone is held and how flat a floor is, sit with the geometry
rather than in a view model. Skipping is always available; levelling then
falls back to inference.

**Verification note.** The engine half is tested on Linux: the fitter, the
verdict policy, the schema round-trip, and the solve's preference for a
measured floor over an inferred one. The Swift half — `FloorCalibrator`, the
capture-view prompt, and the session-writer field — compiles only in the iOS
CI job and has never run on a device, so it is unproven in the way all of
this app's Swift is.

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

## How big a project can get

A house is not one capture. Measured expectations from the intended use:
**200-500 images per room**, a large open living/dining/kitchen split across
more than one capture, and **around ten captures for a house** — so 2,000 to
5,000 images for the project.

Three different limits apply, and they are frequently confused:

| limit | value | what sets it |
|---|---|---|
| frames per capture | **200 target, 500 cap** | what one room needs |
| frames per project | free disk | ~1.3 MB/frame, so 5,000 frames ≈ 6.5 GB |
| frames per **solve** | ~900-1200 | resident feature memory, measured below |

The capture limit is per capture, not per project. A chain shares one world
frame, so a house is ten linked sessions of a few hundred frames each and the
project total is bounded by the phone's free space. The capture UI warns on
**actual free disk** rather than only a frame count, because a capture that
dies on a write halfway through a room loses the room.

Both ends of the capture band are surfaced, and they must not look alike —
one says keep going, the other says stop. Under 200 is ordinary progress, not
a warning; it is also the case people actually hit, and it is invisible
without being told, since a thin capture looks fine at the time and only
shows up as holes in the trained splat hours later.

**What makes 500 enough is that storage is gated on movement, not on a
clock.** It was a flat 0.30 s cadence, which stored 3.3 frames for every
second the camera was running — so pausing to think, or turning on the spot,
spent the room's budget on viewpoints the solve already had. The gate now
also requires 5 cm of travel or 5 degrees of rotation since the last stored
frame, matching the engine's own `store_min_translation_m` /
`store_min_rotation_deg`, with a 3 s backstop for slow deliberate movement.
When there is no pose to compare against — tracking lost, or before bootstrap
— the frame is KEPT: losing a real viewpoint is worse than keeping a
redundant one.

The solve limit is the real one, and it is memory. `final_solve` holds every
frame's features resident from extraction (S1) through track building (S5),
releasing descriptors only afterwards. Measured from the feature cache at
3,000 SIFT features per frame: **1.24 MB per frame**.

| frames | resident features |
|---|---|
| 500 | 0.62 GB |
| 900 | 1.11 GB |
| 2,000 | 2.48 GB |
| 5,000 | 6.19 GB |

iOS will not let an app hold the last two, and that is before tracks, points
and the Ceres problem. So **a 5,000-image project cannot be one on-device
global solve**, and no amount of raising the capture cap changes that — the
two limits are independent, which is exactly why they are now separate
numbers with separate reasons.

What makes the project work anyway is that a global solve is not what splat
training wants. Rooms are trained separately (see split export above), and a
chain already shares one world frame from live localization, so per-session
solves land in the same coordinates. The open question is accuracy at the
seams: live localization is metric to ~3.5 cm, where a bundle-adjusted joint
solve would be millimetric. Whether 3.5 cm shows where two rooms meet is a
measurement nobody has taken yet, and it decides whether a project-level
alignment refinement is needed or is gold-plating.

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

The orbit-based capture walk turned up two more of the same family, and both
are now structural rather than patched:

- **Look targets are blended as bearing, range and height — never as
  points.** Interpolating between "the wall 2.4 m behind me" and "the table
  6 m ahead" sweeps a point across the room and, part-way, straight through
  the camera, where a look target has no direction at all. Same for the
  smoothing window: the midpoint of two targets half a metre apart on the
  path can be the camera itself. Measured, that pointed the lens almost
  straight down for a stretch of an otherwise unremarkable lap.
- **The view rate limit is applied to yaw and pitch, not to the direction
  vector.** Rotating a direction toward its target along the great circle is
  the shortest path on a sphere, and for a turn near 180 deg that path goes
  *over the pole*: the camera obediently pitches down through the floor,
  spins, and comes back up facing the other way, entirely within the
  declared rotation rate. Per-frame pose change 11 deg while every
  consecutive pair of look directions differed by exactly the 1.33 deg cap.
  A hand does not do that — it yaws. There is also a hard 35 deg pitch clamp
  on the planned view, because the camera frame is built by locking roll to
  world up and that construction degenerates as the axis approaches
  vertical.

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

### Replay does not see what the device sees

A synthetic session holds every frame the harness rendered, so `--live`
hands the tracker the full 30 fps. An exported **device** session holds only
STORED frames — roughly 3 fps — so the same code sees ten times the
inter-frame motion. `--decimate N` feeds every Nth frame, which turns that
difference from an asterisk into a measurement. On the two-room walk:

| input rate | scout tracked | scout ATE (rigid) | capture tracked | capture ATE (rigid) |
|---|---|---|---|---|
| 30 fps (1/1) | 85.9% | 0.024 m | 75.4% | 0.035 m |
| 15 fps (1/2) | **99.4%** | 0.018 m | **83.0%** | 0.064 m |
| 6 fps (1/5) | 99.3% | 0.028 m | 49.0% | 0.681 m |
| 3 fps (1/10) | 34.5% | 0.044 m | — (2 poses) | — |

The load-bearing consequence: **`bs_replay --live` cannot validate live
tracking against a real device session.** At the cadence a device actually
stores, the live pipeline does not track — that is not a bug in the tracker,
it is the tracker being fed a tenth of its input. Live-tracking work has to
be validated on synthetic sessions, or on a device.

The Linux handoff for outsized scans is *not* affected, and it is worth
being precise about why: the exported zip carries `live/`, so the final
solve reads the poses the **device** computed at 30 fps (`LoadLivePoses`)
rather than recomputing them. Only re-running the live tracker is broken.

The 1/2 row is unexplained and deliberately not acted on. Halving the input
rate measured *better* than full rate on both passes, which is the opposite
of the trend either side of it. It may be that 3.4 cm between frames is too
little baseline to be worth the extra work, or it may be specific to this
trajectory. It is not a licence to halve the device's feed rate until
someone has found the mechanism.

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

### How ATE is measured, and why the number tripled without anything breaking

Capture-pass pose error appeared to move the wrong way over the same three
changes (0.10 → 0.21 → 0.32 m, rotation 1.4° → 2.7° → 4.7°), each of which
improved coverage. It was the metric.

ATE was computed after pinning the trajectory at its **first tracked pose** —
`T_align = T_live_ref⁻¹ ∘ T_gt_ref`. That is the honest question to ask of a
live tracker, because that frame defines the map's world origin and
everything the user is shown is expressed in it. But it makes the number
sensitive to something that has nothing to do with tracking quality: a small
rotation error at the anchor becomes a large position error far from it
(1° is 17 cm at 10 m). A run that tracks a **longer** stretch of the same
path therefore scores worse on identical per-frame accuracy — which is
precisely what improving coverage does. Worse still for the capture pass,
the anchor lands on the frame where it relocalized into the scaffold, the
least constrained pose in the whole run.

`bs_replay` now reports both. The second fit is rigid over the whole
trajectory (Umeyama with **scale held at 1** — LiDAR sets scale, and a scale
error must stay visible as trajectory error rather than being absorbed by
the alignment):

| two-room walk | anchored | rigid fit |
|---|---|---|
| capture ATE | 0.158 m | **0.035 m** |
| capture rotation | 2.11° | **0.44°** |
| scout ATE | 0.048 m | **0.024 m** |
| scout rotation | 0.37° | **0.25°** |

The capture pass is accurate to 3.5 cm RMSE, 2.8 cm median, 6.1 cm p95. The
gap between the columns *is* the anchor's leverage; on the 60-frame CI sweep,
where there is no lever arm, the two agree to a millimetre and the anchored
rotation is the better of the two.

`EvaluatePass` also writes `live/ate_<pass>.csv` — per-frame error against
ground-truth position — and `scripts/ate_profile.py` reads it. That says
where the error is, which an RMSE cannot: on the capture pass the worst 10%
of frames hold 41% of the squared error (spread, not localized), and the one
hot spot of 0.18 m is at the start of the pass, in the frames after
relocalization before local BA settles. **The doorways are not where the
error is.**

`--check` enforces the rigid bound (0.06 m / 1.0°) alongside the anchored
ones, because a real accuracy regression can hide inside the anchored slack
and cannot hide inside this. Clean scene sits at 0.002 m / 0.22°, hard scene
at 0.010 m / 0.47°.

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

## Why not ARKit, and what it would cost

ARKit is prohibited as a **pose source** by the spec, and that prohibition is
load-bearing rather than ideological. But the trade is worth stating honestly,
because ARKit does offer one thing this project genuinely lacks.

**What ARKit would give us.** `ARDepthData.confidenceMap` — per-pixel depth
confidence, which AVFoundation does not expose at all. The entire
`w_edge · w_range · w_angle` model in `fusion/residuals.h` exists to
*reconstruct* what Apple already computes: inferring fabricated depth at
discontinuities from gradients, rather than being told. ARKit's VIO is also
IMU-fused, so it survives fast turns and blank walls where our image-only
tracker loses lock — which is precisely the remaining 25% of the capture
pass. Plus plane detection (the floor calibration for free) and `ARWorldMap`
(a battle-tested version of the scout scaffold).

**What it would cost.** ARKit poses are optimized for visual stability in AR:
low-latency, IMU-smoothed, and drift-corrected by discrete relocalization
jumps. They are not bundle-adjusted and not photogrammetrically consistent. A
splat trained on them inherits those jumps as geometry error that is invisible
until training finishes. The final solve produces globally consistent
sub-pixel poses — **1.3 mm ATE at 0.22 px** on the clean gate — which is a
different class of output, and is the entire reason this project exists
rather than using any of the several ARKit scanners that already ship.

ARKit also owns the capture session, so GDC-off, stabilization-off,
depth-filtering-off, AF/AE/AWB locking and format selection become its
decisions rather than ours; and its `sceneDepth` is RGB-fused and temporally
smoothed, which conflicts with RAW being an immutable record of what the
sensor measured.

**The middle path: ARKit as a depth-and-confidence provider, poses still
image-first.** ARKit poses never enter the optimization or the export, so the
mandate — which forbids ARKit *tracking* — is intact.

The engine seam for this **exists and is tested**. `DepthImage.confidence`
carries per-pixel sensor confidence (empty when the backend has none), and
`DepthFrame::ConfidenceAt` multiplies it into the geometric model. Two
properties make it safe to land before the backend does:

- **Absent changes nothing.** Every existing session, and any backend that
  cannot supply confidence, behaves bit-identically. Absent means "no
  opinion", never "zero confidence".
- **It can only lower, never raise.** A sensor claiming high confidence
  cannot rescue depth the geometric model rejects. The worst case is a usable
  pixel down-weighted; there is no case where a fabricated one gets trusted.

Multiplied rather than substituted because the two measure different things:
sensor confidence says how sure the return is, `w_range` and `w_angle` say
whether the surface was at a usable range and angle. A confident reading of a
wall at 60 degrees is still a poor constraint.

What remains for the ARKit backend is entirely on the iOS side, and is the
part that **cannot be verified here** — only CI compiles it and CI cannot run
it. The specific unknowns, each of which would change the design:

| question | why it matters |
|---|---|
| Does `ARConfiguration.configurableCaptureDeviceForPrimaryCamera` (iOS 16+) really permit AE/AWB locking? | Photometric consistency is non-negotiable for splat appearance. Without it, ARKit is not usable as the capture path. |
| Is `.sceneDepth` genuinely per-frame, or already temporally fused? | RAW must be an immutable record of what the sensor measured. `.smoothedSceneDepth` is explicitly not that. |
| Does ARKit expose a lens-distortion LUT? | The COLMAP `OPENCV` k1/k2 fit comes from AVFoundation's `lensDistortionLookupTable`. Without it we fall back to PINHOLE and lose distortion modelling. |
| ARKit `sceneDepth` is 256x192 vs AVFoundation's 320x240. | Fewer depth samples per frame, against a better confidence signal. Net effect unmeasured. |

Because of those, the backend should arrive as a **swappable capture source
behind one interface**, with the AVFoundation path left intact — so the field
test can A/B them on the same room rather than betting the capture path on
untested API behaviour.

### ARKit poses for the live view only

There is one further ARKit use that is genuinely safe, and it earns its keep
at exactly the moment the engine cannot help: **rendering the user's position
in 3D while tracking is LOST.**

The live map view and the recovery arrow both run on the engine's own pose,
which by definition does not exist when tracking is lost — precisely when the
user most needs to know where they are relative to what they have already
captured. ARKit's VIO is IMU-backed and keeps producing a pose through fast
turns and blank walls, so a display-only overlay would stay alive across the
gap and go quiet only when ARKit itself gives up.

**The rule: ARKit poses exist in the app's render layer and nowhere else.**
Not through the C ABI, not in RAW, and above all not in `live/poses.jsonl`.

That last one is the specific danger and it is not obvious. `poses.jsonl`
looks like a log — it is named like one and lives in the disposable LIVE
layer — but the final solve READS it (`LoadLivePoses`, `final_solve.cpp`) as
pose initialization for every frame, and as the position source for deciding
which frames a rescan supersedes. An ARKit pose written there would not sit
inertly in a debug file; it would seed the bundle adjustment and decide which
of the user's frames get discarded. That is a one-line mistake with no
symptom until the geometry is wrong.

Two further honesty notes for whoever builds this:

**It needs a frame alignment, and the alignment can be wrong.** The engine's
world frame is anchored at its first keyframe; ARKit's is anchored at session
start and gravity-aligned. Displaying an ARKit-derived camera position beside
engine-derived geometry means estimating a transform between the two from the
frames where both exist. During a tracking loss that transform cannot be
updated, so it is only as good as it was at the moment of the loss. Over the
seconds a loss usually lasts that is fine; over a long one it drifts, and a
confidently-wrong position overlay is worse than a frozen one — the same
argument that governs the recovery arrow. It should fade out rather than lie.

**The real risk is not technical, it is the temptation gradient.** Once ARKit
poses exist in the app in the same coordinate frame as the engine's, the
distance to "just use them while tracking is lost" or "use them to seed
relocalization" is one small, reasonable-looking commit. That is how a
mandate erodes — not by decision but by convenience. The mitigation has to be
mechanical rather than cultural: a distinct type that cannot be passed where
an engine pose is expected, and a CI gate that permits ARKit's depth and
camera-image symbols while still failing on `camera.transform` reaching
anything but the renderer.

## Camera: locked to the wide, by device not by depth

`AVCaptureDevice.default(.builtInLiDARDepthCamera, ...)` is a virtual device
with exactly two constituents — the **wide** camera and the LiDAR. The
ultra-wide (0.5x) is a separate physical device and Apple ships no
LiDAR-fused virtual device containing it, so **0.5x is unavailable whenever
synchronized depth is required.** This is not a depth-format restriction; the
lens is simply not part of the device.

`videoZoomFactor` is never set, so capture runs at native wide. Zooming in is
possible and undesirable: it crops the FOV, and wide FOV is what supplies
overlap and well-conditioned triangulation. Digital zoom would also change
effective intrinsics per frame, against the one-COLMAP-camera-per-session
model the export depends on.

Getting 0.5x would mean a second capture input on `.builtInUltraWideCamera`
with no depth, exported as a separate COLMAP camera — real work, and it
forfeits LiDAR anchoring on those frames, which is what carries blank walls.

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

(Every ATE in this section and the ones above it is the **anchored** figure,
which is what was being measured at the time. Do not compare them against the
rigid-fit numbers — see "How ATE is measured" above.)

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

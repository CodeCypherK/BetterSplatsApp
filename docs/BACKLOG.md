# Working backlog

Live worklist for autonomous sessions. The container is ephemeral and each
wake-up re-clones, so **this file is the memory** — read it first, update it
last, and push every change. Anything not written here did not happen.

Rules of engagement for this backlog:

- One small, complete, committed change per session. Never leave the branch
  broken.
- `ctest` green before every push. `scripts/validate_colmap.py` before any
  push that touches the solver, the export, or the session format.
- Measure before and after, and record the numbers here. A change with no
  number attached is not done.
- If a measurement contradicts something written here, correct the entry.
  Being wrong in the log is fine; leaving it wrong is not.

## Goal

Cleanest possible reconstruction data, and a capture experience that gets a
non-expert to that data on the first try.

---

## Now

- [ ] **Field-test the whole opening minute on a device.** Four things are
      built and none has met a real room. In order of what breaks the session
      worst if wrong:
      1. **Does the photometric lock land on the scene?** Start a capture,
         let the floor step finish, then look at something 5 m away — is it
         sharp? This is the change most likely to be subtly wrong, because
         the one-shot auto modes' revert-to-locked behaviour is documented
         but unverified here, and a failure is invisible until the splat
         comes out soft.
      2. **Floor calibration**: does the verdict wording guide someone to a
         usable floor, does the accepted frame survive into the final solve
         (`report.json` -> `floor_measured: true`), and is one step forward
         enough movement for it to register.
      3. **The scout circuit**: is "back to the walls, camera facing in"
         followable, and does the capture pass actually relocalize into what
         it left behind.
      4. **Weak-area guidance**: with the live pose wired in, do the
         distances and left/right calls match where the user is standing.

- [x] ~~**Floor-calibration capture UI.**~~ The engine side is done and tested:
      `FitDepthPlane` measures the plane from one depth frame, session.json
      carries it in camera coordinates, and the solve prefers it over
      inference (measured: cameras at 1.496 m vs 1.447 m inferred, truth
      1.5). What is missing is the prompt — "point at the floor" — plus the
      confidence readout and a retry when the fit refuses. Two constraints
      the UI must respect, both learned the hard way: the calibration has to
      name a frame that is genuinely stored in the session, and the user has
      to be MOVING while they sweep the phone up, or those frames have no
      parallax and the solve drops them.

- [x] ~~**Capture-pass ATE keeps rising as coverage rises.**~~ **Answered:
      it was the metric, not the tracker.** The ATE was anchored at the
      first tracked pose, so any rotation error there scales with distance
      travelled — 1 deg is 17 cm at 10 m — and a run that tracks a LONGER
      stretch scores worse on identical per-frame accuracy. That is exactly
      what three consecutive coverage improvements did. Worse, the anchor
      lands on the capture pass's *relocalization* frame, which is the least
      constrained pose in the whole run.

      Measured on the two-room walk (`bs_replay` now prints both):

      | | anchored | rigid fit |
      |---|---|---|
      | capture ATE | 0.158 m | **0.035 m** |
      | capture rot | 2.11 deg | **0.44 deg** |
      | scout ATE | 0.048 m | **0.024 m** |

      So the capture pass is accurate to **3.5 cm RMSE, 2.8 cm median, 6.1 cm
      p95** — not 16 cm and never 32 cm. `scripts/ate_profile.py` reads the
      per-frame dump: error is **spread, not localized** (worst 10% of frames
      hold 41% of the squared error), with one hot spot of 0.18 m at the
      start of the pass, i.e. the frames right after relocalization before
      local BA settles. The doorways are not the problem.

      **Use the rigid number for any comparison.** `--check` now enforces it
      (0.06 m / 1.0 deg) alongside the anchored bounds; clean sits at
      0.002 m / 0.22 deg, hard at 0.010 m / 0.47 deg.

- [ ] **Capture-pass coverage: 75.4% tracked, and that is the real target
      now that accuracy is understood.** The relocalization anchor bug took
      it from 12% to 62%; trajectory reshaping to 75%. What is left is
      genuine: the pass still loses tracking at the doorway transits, where
      the turn outruns mapping (see ARCHITECTURE.md). Two geometry-side
      attempts both measured **worse** and are reverted — denser keyframes
      while turning (61.9% → 38.9%) and baseline-aware triangulation
      partners. Together they say the constraint is not geometry: it is
      MATCHES. A turn shows the camera surfaces at viewpoints ORB cannot
      match, and no keyframe arrangement fixes that. Next thing worth trying
      is on the matching side, e.g. re-detecting at a lower FAST threshold
      while turning, or matching the leading edge against the frame before it
      rather than only against keyframes.
      NOTE: the old "scaffold is appearance-incompatible / ORB is
      viewpoint-sensitive" theory here was **wrong** and is disproved — the
      same scaffold now carries 75% of the pass. Do not rebuild the scout
      trajectory on that basis without a fresh measurement.

## Next

- [ ] **Should the scout circuit store frames as densely as capture does?**
      It currently shares the 0.30 s gate, so a one-minute lap spends ~200 of
      the 900-frame budget and ~260 MB on frames the final solve throws away.
      Thinning them frees that for the scan. Against: replay only ever sees
      STORED frames, so a sparser scout makes replay's scout pass much harder
      than the device's ever was, and that gap is already the weakest part of
      the dev loop. Do not guess an interval — measure scout tracking against
      stored-frame cadence first.
- [ ] **Two-room walkthrough as a CI gate.** Blocked on render cost
      (~2377 frames at walking pace ≈ 13 min to generate, ~7 min to replay).
      Consider a shorter realistic path, or a cached fixture committed to
      `core/testdata/`.
- [x] ~~**Capture UX for the scout pass.**~~ Built: a plan chooser asks "one
      room / several rooms" before the camera starts, the circuit runs as
      `pass="scout"`, and "Start detailed scan" ends it and begins the
      capture pass in the same session. Found while building it that
      `FrameMetaJSON` had no `pass` field at all — every frame the app wrote
      said "capture", so a scout circuit captured through the app would have
      been reconstructed from.
- [x] ~~**Readiness guidance wording pass.**~~ Done, and it turned up a real
      bug: the dashboard printed `|centroid|` as "N m away", which is the
      distance from the session origin, not from the user. Distances now go
      through the live pose, the engine's long-computed move direction
      finally resolves to left/right/forward/back, and the five messages lead
      with the action instead of the diagnosis.

## Needs a real device session

These cannot be settled on synthetic data. Each names what to look for.

- [ ] **Does frame-selection-by-sharpness help?** `bs_replay --live` prints
      `storage: kept N of M frames, sharpness X vs Y sequence mean`. On a
      real handheld walk that number should be clearly positive; on synth it
      is 0.4%. If it is not positive on device, drop the gate.
- [ ] **Is blur intermittent or smooth on a real walk?** Plot `lap_var`
      across a device session. The synthetic model assumes smooth-with-speed;
      if real captures spike at footfalls, the harness should model that —
      with an amplitude measured from the session, not guessed.
- [x] ~~**Does AF/AE/AWB lock on the right thing?**~~ It did not, and the
      floor calibration made it certain rather than merely likely: the lock
      fired on a 1.5 s timer from `start()` while the prompt was telling the
      user to point at the floor. Now it fires when the floor step ends
      (accepted, skipped, or given up on), converges via the one-shot
      `.autoFocus`/`.autoExpose`/`.autoWhiteBalance` modes with a 4 s
      backstop, and reads the resulting mode back off the device instead of
      assuming the assignment stuck. **Still needs a device session** to
      confirm the modes behave as documented and 4 s is enough.

## Ideas (unranked, unvalidated)

- Exposure/white-balance normalization across frames before the final solve.
- Per-session depth-vs-triangulation affine correction (risk 6 in the plan).
- Report per-image blur/exposure outliers in `report.json` so a user can see
  which frames hurt the model.

---

## Log

Newest first. One line per session: what changed, what it measured.

- **The capture-pass ATE regression was never real.** Anchoring the
  trajectory at its first tracked pose multiplies that pose's rotation error
  by distance travelled, and the anchor lands on the relocalization frame —
  the least constrained pose in the run. Rigidly aligned (Umeyama, scale
  held at 1, because LiDAR sets scale and a scale error must stay visible):
  capture **0.035 m / 0.44 deg** against 0.158 / 2.11 anchored; scout 0.024
  against 0.048. Per-frame dump says the error is spread, not at the
  doorways. `--check` now enforces the rigid bound too, which is the
  sensitive one.

- **Two-pass capture UX + the pass tag that made it real.** `FrameMetaJSON`
  had no `pass` field, so every frame the app wrote said "capture" — a scout
  circuit captured through the app would have been reconstructed from. The
  floor calibration had to move to the capture pass for the same reason: the
  solve excludes scout frames, so a floor filed against one names a frame
  that never gets a pose.

- **AF/AE/AWB locked on a 1.5 s timer from camera start**, while the floor
  prompt was actively telling the user to point at the floor — a whole
  session focused at ~1 m and exposed for a patch of floor, with no
  recovery by design. Now locks when the floor step ends, on convergence
  rather than a clock.

- **The dashboard was quoting distances from the session origin.** It took
  the length of a weak area's world-space centroid and printed "3.2 m away",
  which is 3.2 m from where the session started. Everything spatial now goes
  through the live pose.

- **Floor calibration**: measure the floor from one LiDAR frame at capture
  time instead of inferring it afterwards. 17k inliers at 11 mm rmse vs a
  few hundred sparse points at 23 mm. Cameras land at **1.496 m** (truth
  1.5) against 1.447 m from inference. Stored in camera coordinates + frame
  id so the world plane is derived from the final pose. Two wrong turns:
  filing the plane against a frame whose real pose differed put the floor
  1.1 m out, and a pure-rotation calibration sweep has no parallax so the
  solve registered none of it (71/84 -> 84/84 once the sweep moves).

- **Levelling (S8b)**: find the floor, level it to y=0, square the dominant
  walls to the axes — one rigid transform over poses and points, applied
  before the dense cloud so everything downstream inherits it. Clean solve:
  **cameras at 1.494 +- 0.011 m** against a true 1.5. Two-room solve (0.2 m
  geometry error): 1.12 +- 0.41 m, so the report carries camera height and
  spread as the quality signal. Three wrong turns worth remembering: the
  ceiling passes every floor test if you only check "nothing beyond it"
  (picked it, cameras landed at 0.97 m); blind RANSAC finds nothing on a
  real reconstruction because floors are a small share of tracked points;
  and the rotation angle is not a quality signal (168 deg on a good scan).
- Split overlap widened (8 -> 3 shared points): seams went from 13 to 36
  shared images. You can see a long way into a room from the one next door,
  and those oblique views are what make two separately trained splats agree
  along their shared boundary.

- **Split export** (`final_split_max_images`): writes `final/colmap_parts/`
  beside the combined model so a large facility can be trained a room at a
  time. Parts follow the covisibility graph — doorways are the weak edges,
  so that is where the cuts land. Measured on the two-room walkthrough: 100
  images into 2-3 parts, 13-29 images shared at the seams, **pose delta vs
  the combined model exactly 0** (pycolmap, in the CI gate now). Two bugs
  found by running it: absorption never fired (stale union-find roots), and
  image count alone let through a part of 17 images holding 3 points.

- Storage gate now prefers sharp frames (it was purely geometric, so a
  smeared frame landing on the 5 cm boundary went into immutable RAW and the
  final solve reconstructed from it). Guarded: stores regardless past 2x the
  geometric threshold, so coverage never suffers. **Unverified** — three
  attempts to measure it failed because synthetic blur moves lap_var far
  less than the procedural texture does (1.2x spread either way, stored
  sharpness within 0.4% of the mean with the gate on or off). A footfall
  blur term was tried to create variance, did not, and broke the hard-scene
  bound via a guessed amplitude — reverted. Needs a device session.
  `bs_replay` now reports what the gate selected, which is how any of this
  became visible.

- Reshaped both two-room paths so they cross the doorway **square to the
  opening** instead of cutting in diagonally from a far corner and pinching
  to a point (user spotted it in the plan view — the shape should read as an
  H). Widened the opening 1.1 m -> 1.6 m. Scout tracking **89.3% -> 98.6%**,
  scout ATE 0.080 -> **0.043 m**; capture 72.5% -> 73.2%. Capture ATE keeps
  drifting up (0.21 -> 0.32 m) as coverage rises — still unexplained.

- Scout now looks into the room it is ENTERING while crossing a doorway,
  rather than at the doorway itself (user's call, and it holds up).
  Capture-pass tracking **61.8% -> 72.5%**, scout flat at 89.3%. The lead
  distance is bounded on both sides: 5 m put the switch back on the wall leg
  where the camera walks toward what it looks at, and scout tracking
  collapsed to 33%; 3 m puts it on the approach leg, travel across the view.
  Capture ATE rose 0.10 -> 0.21 m, partly a selection effect (harder poses
  now counted) but not separated out.

- Tried baseline-aware triangulation partners (require a partner >= 4% of
  scene depth away). **Negative, reverted**: capture tracking flat
  (61.9% → 61.8%) but ATE 0.075 → 0.101 m and 4% fewer points; scout ATE
  0.038 → 0.047 m. Taken with the cadence result, this kills the
  "short baselines make weak points" story — requiring LONG baselines hurts
  too. Both changes cost matches. Overlap beats parallax here; the live map
  runs on track extensions.

- Tried rotation-scaled keyframe cadence (interval capped so consecutive
  keyframes stay within 5 deg of turn). **Negative result, reverted**:
  capture pass 61.9% → 38.9%, ATE 0.075 → 0.223 m; scout roughly flat
  (89.5% → 89.4%) though its ATE improved 0.038 → 0.030 m. Denser keyframes
  shorten baselines, and triangulation pairs with the most covisible (=most
  recent) neighbours. Recorded in the ruled-out table.

- **Relocalization never moved the local-map anchor.** `Relocalize` set the
  pose but not `last_kf_id_`, so each recovery handed the next frame a local
  map for the place the camera had just left — reloc-lose-reloc-lose.
  Capture pass **12.2% -> 61.9%** tracked, ATE **0.218 -> 0.075 m**, reloc
  thrash 24 -> 7 events; scout 86.0% -> 89.5% and now ends TRACKING rather
  than LOST. Corrects the earlier "ORB viewpoint sensitivity" diagnosis,
  which was wrong.

- Verified the CI gate against the reshaped trajectories (they changed
  `bs_synth` output, and the previous push had not run it): live 95% tracked
  / 3 mm ATE, final **60/60 registered, 1.3 mm ATE, 0.22 px**, hard scene
  60/60 and 5440 points. Green. Extracted `TurnRateDps` beside
  `MotionIsPlausible` and covered it: 1 deg/frame = 30 deg/s (quiet),
  4 deg/frame = 120 deg/s (warns, but stays plausible and is *not* rejected —
  warning and rejecting are separate jobs).

- `aed20fa` Harness made physically honest (`--speed`/`--pan`/`--fps`, worst-frame
  motion reporting); fixed a 174 deg doorway view flip and an ill-conditioned
  look target. Scout circuit 33% → **86%** tracked, ATE **4.3 cm**, scaffold
  85 → **224** keyframes. Added `SLOW DOWN` above 60 deg/s; per-pass pose logs;
  per-test temp dirs so `ctest -j` passes.

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

- [ ] **Do the seams show between separately-solved sessions?** A house is
      ~10 captures / 2,000-5,000 images, which **cannot be one on-device
      global solve** — resident features measure 1.24 MB/frame, so 5,000
      frames is 6.2 GB before tracks, points or Ceres (table in
      ARCHITECTURE.md). Rooms are trained separately anyway, and a chain
      already shares one world frame, so per-session solves land in the same
      coordinates — but only to live-localization accuracy, ~3.5 cm, where a
      joint solve would be millimetric. **Measure whether 3.5 cm is visible
      where two rooms meet before building anything.** If it is, the fix is a
      project-level alignment refinement over cross-session matches; if not,
      that work is gold-plating.

- [ ] **Should the scout circuit store frames as densely as capture does?**
      It currently shares the 0.30 s gate, so a one-minute lap spends ~200 of
      the 900-frame budget and ~260 MB on frames the final solve throws away.
      Measured now (`--decimate`, table in ARCHITECTURE.md): stored cadence
      does **not** affect on-device scout quality at all — the device feeds
      the tracker every frame at 30 fps regardless of what it stores. It only
      affects replay fidelity and disk. And replay of the scout pass at
      device cadence is already 34.5% tracked, i.e. already broken, so
      thinning costs little that is not already lost. Leaning toward thinning
      the scout gate; wants one more look at whether anything else reads
      scout frames first.

- [x] ~~**Why does 1/2 decimation track BETTER than full rate?**~~
      **It does not, reliably. Closed — do not halve the device feed rate.**
      Seed 7 showed scout 85.9% -> 99.4% and capture 75.4% -> 83.0% at half
      rate. Seed 11 does not reproduce it:

      | | seed 7 | seed 11 |
      |---|---|---|
      | scout tracked, 1/1 -> 1/2 | 85.9% -> 99.4% | 85.9% -> **94.4%** |
      | capture tracked, 1/1 -> 1/2 | 75.4% -> 83.0% | 74.8% -> **74.5%** |
      | scout rigid ATE, 1/1 -> 1/2 | 0.024 -> 0.018 | 0.022 -> **0.045** |
      | capture rigid ATE, 1/1 -> 1/2 | 0.035 -> 0.064 | 0.044 -> **0.024** |

      The capture-pass gain was seed-specific and vanishes; the scout gain
      reproduces in direction but half the size, and seed 11's scout ATE gets
      **worse**. Every column disagrees with its neighbour. This was one
      trajectory's luck, and refusing to act on a single seed was correct.
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

- [ ] **Are the per-image quality thresholds any good?** `report.json` now
      carries a per-image table and flags (blurry / overexposed /
      weakly_observed / unregistered). Two of the four **cannot fire on
      synthetic data at all**: `lap_var` spans 1.38x across the whole hard
      scene (119..164, so nothing reaches the 0.5x-of-median floor) and
      `overexp_frac` is exactly 0 on every synthetic frame. The logic is
      unit-tested; the numbers are a documented guess. On a device session,
      plot the lap_var distribution and check the flagged frames are the ones
      that actually look bad. Low risk to be wrong — it labels a report, it
      does not touch the reconstruction — but a list that cries wolf gets
      ignored on the session where it matters.

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

- **`report.json` carries no readiness regions**, though the plan specifies
  per-region scores and weak clusters recomputed from FINAL data. The final
  solve never builds a PatchGrid — readiness only exists live. Would need a
  LiveMap assembled from the solve's frames+tracks. Wanted for: room bounds
  the user could pick from, and a post-solve room-by-room quality view.

- Exposure/white-balance normalization across frames before the final solve.
- Per-session depth-vs-triangulation affine correction (risk 6 in the plan).
- Surface the flagged images in the app, not just in `report.json` — a
  "these 12 frames are hurting your model" list on the Processing or Export
  screen, with the option to exclude and re-solve.

---

## Log

Newest first. One line per session: what changed, what it measured.

- **Gap-closing pass.** `docs/FORMATS.md` calls itself normative and was
  missing five keys that already ship — `floor_calibration`, `project_id`,
  `project_name`, `parent_session`, `supersedes` — plus everything in
  `live/` beyond `poses.jsonl`. Documented, with the chain rules and what
  each malformed case does. Also fixed the Swift 6 isolation violations in
  `SessionStore` (they are warnings today, errors under Swift 6): `UIDevice`
  is MainActor-isolated and an actor's init is not, so the OS version is now
  passed in; and the isolated `writeSessionJson()` was being called from
  that init, so the write path is static.

- **Project board, and the rescan loop closed.** Create a project, reopen it,
  capture another room, re-export, redo a room. The chaining and supersession
  built earlier were unreachable until now: every capture numbered frames
  from 1, so two chained captures would collide and the reader would reject
  them. Ids now continue from the highest one on DISK (not `frame_count`,
  which is 0 in a capture killed before finalize).

  **The rescan volume is OBSERVED, not predicted** — the box the camera
  actually walked, grown by 1.5 m because the camera stands in the middle of
  a room and looks outward, so the box of where the feet went is much smaller
  than what was re-covered. This avoided a much bigger piece of work: the
  final solve computes no readiness regions at all, so there are no stored
  room bounds to pick from. Observing is also self-correcting and does not
  depend on bounds from a solve two versions ago.

- **Rescanning a room.** A project is a chain of sessions over one space;
  going back to redo a room writes a new session declaring the world volume
  it re-covered, and the solve declines to reconstruct from earlier frames
  inside it. Same principle as scout frames: RAW is untouched, "not
  reconstructed from" is a solve-time decision, and deleting the volume
  brings the old room back byte for byte. Measured end to end on a split
  60-frame session: **15 superseded frames excluded, 3 unlocatable ones
  kept, 45/45 registered, all 60 frames still on disk**. Position comes from
  the live poses, which share one world frame across the chain — that is
  what makes a volume from one session mean anything to another. A frame the
  live pass never posed cannot be located and is KEPT: discarding data we
  cannot place would be the more damaging mistake.

- **Session chaining: a facility bigger than one capture.** The app caps at
  900 stored frames (~5 min) and there was no way to join two captures, so
  the stated use case — large facilities, many rooms — could not be captured
  at all. `session.json` now carries `parent_session`, and `SessionReader`
  resolves a chain transparently: `frame_ids()` spans the whole chain and
  every accessor resolves an id to whichever session holds it. **Nothing
  downstream changed** — verified by splitting a 60-frame session in two and
  solving the child: 60/60 registered, 7237 points, 0.22 px, ATE 1.3 mm, and
  **max pose delta against the unsplit solve exactly 0**. A capture pass now
  also writes `live/map_end.bin` for the next session to localize into
  (deliberately not `map.bin`, which would make a re-run start from the
  previous run's result). Seven tests pin the malformed-chain cases, all of
  which are silent corruption rather than crashes: duplicate frame ids,
  self-reference, cycles, missing parent, and a moved chain.

- **ATE finding reproduced on a second seed.** Seed 11 two-room walk: scout
  85.9% tracked / 0.022 m rigid (seed 7: 85.9% / 0.024), capture 74.8% /
  0.044 m rigid (seed 7: 75.4% / 0.035). Anchored:rigid ratio 5.2x against
  seed 7's 4.5x. The anchoring artifact is not seed-specific.

- **The app now reads `report.json`.** Everything the engine measures about
  a finished reconstruction was dying in a file nobody was going to open,
  one screen before the export buttons — which is exactly where "train on
  this or rescan?" gets decided, and it is far cheaper to decide there than
  after an hour of GPU time. Verdict, plain-language summary, concrete
  advice, and a drill-down naming the flagged photos. The drill-down says
  whether they cluster into a few stretches of the walk (go redo those) or
  are spread across the session (light level or pace), because those need
  opposite responses. `validate_colmap.py` now parses report.json and
  cross-checks the per-image table against the pycolmap model, so the
  Swift/C++ contract is enforced on every push instead of on a phone.

- **`report.json` now names the frames that hurt the model.** Per-image
  table (registered, observations, reproj rmse, lap_var, overexp_frac) plus
  four flags, with thresholds relative to the session's own distribution —
  lap_var is not comparable between scenes, so an absolute cutoff would
  condemn every frame in a plain room and none in a busy one. Residuals use
  the solver's own convention so the number reported is the one the
  optimizer saw. Verified end-to-end on the hard scene: valid JSON, 60
  images, median lap_var 150, median 675 observations, zero false positives.

- **TRACKING LOST now says which way the map is.** `guide_dir`,
  `guide_dist_m` and `guide_region_id` had existed in `bs_api.h` since M4
  and nothing had ever written or read them, so the pill said "return to a
  mapped area" — the one fact the user is missing — while the engine held
  the answer. Direction to the nearest keyframe, in camera coordinates so it
  needs no valid pose, drawn as an arrow. Nearest rather than most recent,
  and suppressed under 0.35 m where the problem is aim rather than
  position. Seven sign-and-frame tests; the first version of one of them had
  the rotation backwards, which is the whole reason the geometry is a free
  function instead of a method.

- **`bs_replay --live` cannot validate live tracking on a real device
  session, and now we know by how much.** `--decimate N` feeds every Nth
  frame; at the ~3 fps a device stores, the live pipeline tracks 34.5% of
  the scout pass and essentially none of the capture pass. That is not the
  tracker failing, it is the tracker being fed a tenth of its input — synth
  sessions hold all 30 fps, device sessions hold only stored frames. Live
  work has to be validated on synth or on a device. The Linux handoff for
  outsized scans is unaffected: the exported zip carries `live/`, so the
  final solve reads the poses the device computed rather than recomputing
  them.

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

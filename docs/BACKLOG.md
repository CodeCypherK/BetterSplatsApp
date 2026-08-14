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

- [ ] **Capture-pass tracking: 62% and worth pushing further.** The
      relocalization anchor bug (below) took this from 12% to 62% with ATE
      0.218 -> 0.075 m. What is left is genuine: the pass still loses
      tracking at the doorway transits, where the turn outruns mapping (see
      ARCHITECTURE.md). Two geometry-side attempts both measured **worse**
      and are reverted — denser keyframes while turning (61.9% → 38.9%) and
      baseline-aware triangulation partners (ATE 0.075 → 0.101 m). Together
      they say the constraint is not geometry: it is MATCHES. A turn shows
      the camera surfaces at viewpoints ORB cannot match, and no keyframe
      arrangement fixes that. Next thing worth trying is therefore on the
      matching side, e.g. re-detecting at a lower FAST threshold while
      turning, or matching the leading edge against the frame before it
      rather than only against keyframes.
- [ ] **Capture-pass ATE regressed to 0.21 m while coverage rose to 72.5%.**
      Separate the selection effect from a real accuracy loss: compare ATE
      over the frames tracked by BOTH runs before concluding anything.
      NOTE: the old "scaffold is appearance-incompatible / ORB is
      viewpoint-sensitive" theory here was **wrong** and is disproved — the
      same scaffold now carries 62% of the pass. Do not rebuild the scout
      trajectory on that basis without a fresh measurement.

## Next

- [ ] **Capture UX for the scout pass.** There is no UI for "walk the
      perimeter first" — the pass exists in the engine and the format but a
      user cannot invoke it. Design the flow, then build it.
- [ ] **Readiness guidance wording pass.** Messages are generated from
      sub-score argmin; check they read as instructions a stranger can
      follow, not as diagnostics.
- [ ] **Two-room walkthrough as a CI gate.** Blocked on render cost
      (~2300 frames at walking pace ≈ 13 min). Consider a shorter realistic
      path, or a cached fixture committed to `core/testdata/`.

## Ideas (unranked, unvalidated)

- Exposure/white-balance normalization across frames before the final solve.
- Per-session depth-vs-triangulation affine correction (risk 6 in the plan).
- Report per-image blur/exposure outliers in `report.json` so a user can see
  which frames hurt the model.

---

## Log

Newest first. One line per session: what changed, what it measured.

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

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

- [x] ~~**Capture-pass coverage on the circle-and-orbit walk: 62.4%
      tracked.**~~ **Gone. Re-measured at 100.0%, and nothing was built to
      fix it.** The plan was to seed relocalization with the scaffold
      keyframes nearest the scout's last pose instead of sweeping the map —
      "a search-order problem with a known answer". Measuring first was the
      right call: there is no longer anything to fix. Seed 7, scout +
      capture, 3370 frames:

      | | recorded (62.4%) | now |
      |---|---|---|
      | scout tracked / rigid ATE | 85.9% / 0.024 m | **99.4%** / 0.021 m |
      | capture tracked | 62.4% | **100.0%** |
      | capture rigid ATE | 0.029 m / 0.41 deg | **0.019 m** / 0.24 deg |
      | capture anchored ATE | 0.067 m | **0.023 m** |
      | keyframes / points | 675 / 32.5k | **921 / 42.9k** |

      The credit belongs to the two trajectory changes the user asked for,
      not to anything in the tracker: hugging the perimeter and looking
      ACROSS the room means the capture pass now starts 0.4 m from where the
      scout finished and pointed within a couple of degrees of the same
      bearing, so the five newest scaffold keyframes match on frame one.
      The old lap looked 45 deg along the wall from further out, which is a
      different picture of a different surface, and 340 frames of sweeping
      the map was the price. **Anchored and rigid ATE have converged**
      (0.023 vs 0.019), which is the signature of a first pose that is
      actually well constrained — the whole anchoring artefact was an
      artefact of relocalizing badly.

      Also note the capture pass alone, with no scout at all, tracks 99.8%
      at 0.019 m rigid. The scaffold is no longer carrying the run; it is
      insurance.

- [x] ~~**Capture-pass coverage: 75.4% tracked, and that is the real target
      now that accuracy is understood.**~~ **Closed at 100.0% — see the entry
      above.** (Measured on the OLD 33 m walkthrough; kept only for its
      analysis, which is still worth having: the two things it ruled out are
      still ruled out.) The relocalization anchor bug took
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

- [x] ~~**Five `bs_live_status` fields the app reads from nothing.**~~
      **Closed, and one of them was worse than unread — it was never
      written.** `px_error_mean` was published as a tracking-health number
      since M4 and no code ever assigned it: every poll reported exactly
      0.0. The engine now computes it from the reprojection error of the
      PnP inliers, while they are still in hand.

      What each field got, and why:
      - `px_error_mean` + `inlier_ratio` -> `bs_replay`'s live summary, as
        **tracking health, which is a different question from tracking
        coverage**. The tracked percentage says how often PnP succeeded;
        these say how comfortably. A pass tracking 99% on a thin inlier
        ratio at 1.5 px is one bad frame away from losing it, and is
        indistinguishable in the coverage number from one tracking on 200
        inliers at 0.4 px. Clean scene reads `median 97% inliers at 0.64 px`.
      - `guide_region_id` -> the TRACKING LOST hint now names the room it
        points at, when the session has more than one to mean. The engine
        had been computing it since M4 and the app drew an unnamed arrow.
      - `blur_metric` -> **deleted from the ABI**. The app computes its own
        `lap_var` in FrameAnalysis and writes it to meta.json; a second copy
        across the boundary was redundant rather than merely unused.
      - `last_frame_id`, `map_points` -> already read, by `test_api` and
        `bs_replay` respectively. An app-only audit had missed them, which
        is worth remembering: the app is not the only client of this ABI.

      Original entry kept below for the failure mode it names. The same
      audit as the config knobs, one layer up, and the same failure mode: the
      engine measures something, publishes it across the ABI, and the app
      drops it — which is exactly how `keyframe_ids` came to be empty in
      every session ever captured. Unconsumed today:
      `inlier_ratio` and `px_error_mean` (the engine's own tracking-health
      numbers; the UI shows only the guidance pill), `blur_metric` (the app
      computes its own in FrameAnalysis, so this one is genuinely redundant
      and could be removed from the ABI instead), `last_frame_id`,
      `map_points`, `guide_region_id`.
      `store_spacing_m` left the list when the store gate needed it.
      **`frames_processed` is gone, and it was worse than unread** — it was
      documented as "fed minus dropped" and incremented in lockstep with
      `frames_fed`, inside the same mutex, so it could never report a drop.
      `bs_live_feed` tracks inline on the caller's thread; the engine cannot
      drop a frame it was handed. The drop is the app's, and the app now
      counts it (see the log entry on EngineFeeder).
      Worth a `check_abi_used.py` in the same spirit, though it is harder:
      a Swift field read is not a grep away from a C struct member.

- [ ] **Four pieces of documented behaviour that were never built.** Found by
      `scripts/check_config_used.py`, now a CI gate: every `EngineConfig`
      field must be read somewhere in the engine, and eight were not. Two were
      implemented at the time (the floater sweep's radius-outlier pass); two
      more have been since; one of the eight turned out to be a **mistake in
      this list** and is corrected below. What is left:
      - **`live_max_keyframes = 600`** — the live map has NO keyframe cap,
        and it is now measured rather than feared. The two-room walk builds
        **921 keyframes / 42.9k points, serializing to 41 MB**; call it
        ~63 KB per keyframe on disk and rather more resident. Keyframes scale
        with PATH, not with frames or with stored frames, so the app's
        500-frame session cap does not bound this: a five-minute walk at
        1 m/s is ~300 m, about 2.5x this fixture, so ~1,600 keyframes and
        ~100 MB serialized. Not fatal on the target device, and a chained
        session loads only its immediate parent's map (one level, not the
        whole chain), so the ceiling is roughly two sessions' worth.
        **The reason not to just add the cap is that culling keyframes costs
        exactly what the scaffold is for** — a capture pass relocalizes into
        those keyframes, and the ones a cap would drop are the oldest, which
        is to say the first room. If this ever needs a bound, bound it by
        BYTES and cull by redundancy (a keyframe whose points are nearly all
        seen by others), not by age.
      - **`final_bow_top_k = 10`** — appearance retrieval for the pair graph
        (plan stage S2). The graph is index-only. Not costing anything
        measurable today (SIFT connects the walk on strides alone) but it is
        the reason a revisit hundreds of frames later is never proposed.
        Track completion was tried as a substitute — it borrows candidates
        from the nearest cameras in SPACE, which reaches revisits an index
        stride never will — and it made the reconstruction worse (entry
        below). Retrieval is still the right shape for this: it proposes
        pairs BEFORE any pose exists, and those pairs then go through the
        same geometric verification as every other, rather than being
        adopted on a projection.
      - **`final_max_pairs_per_image = 40`** — pair cap. Moot at current
        stride counts (~18/image) but real under `final_exhaustive_below`,
        where a 140-image fixture matches 139 pairs per image and takes 20
        minutes.
      - **`live_queue_depth = 2`** — bounded frame queue. This was filed as
        "arguably belongs to the app rather than the engine; the app already
        applies backpressure", and that dismissal was **wrong on both
        halves**: the app's backpressure covered STORAGE, nothing bounded the
        engine feed, and feeding happened inline on the capture delegate
        queue. It belongs to the app, which is where it now is
        (`EngineFeeder`, one slot, drop-oldest, counted) — but it was a real
        gap, not a misfiled knob.

      Done since:
      - ~~**`final_track_complete_px = 6.0`**~~ — **built, measured, and
        removed. It makes the reconstruction worse.** The plan puts track
        completion in S8 and the reasoning is appealing: once a track has a
        point and a frame has a pose, where it lands in that frame is
        arithmetic, so look there among unclaimed features. Implemented with
        candidates drawn from the twelve nearest cameras BY POSITION, which
        also reaches revisits the index-strided pair graph never proposes.
        Two attempts, on the 400-frame SIFT fixture:

        | | baseline | geometry only | descriptor-gated |
        |---|---|---|---|
        | registered | 377/400 | 378/400 | 377/400 |
        | points | **80,352** | 74,608 | 79,518 |
        | rmse | **0.32 px** | 0.89 px | 0.37 px |
        | mean track | 4.8 | 5.9 | 4.9 |
        | ATE | **0.057 m** | 0.298 m | 0.104 m |

        The first attempt had no working appearance check at all — for SIFT
        the absolute cap is disabled and a ratio test inside a six-pixel disc
        passes on anything, because a disc almost always holds one candidate.
        The second derives an absolute threshold from the session's own
        verified inliers (90th percentile, 0.3 in RootSIFT L2), which cut
        completions 9x, from 70k to 8k in round one — and it is STILL worse
        than not doing it: ATE 1.8x, rmse up, marginally fewer points, and
        mean track length barely moved.

        Best explanation: candidates come from the nearest cameras, so every
        completed observation is a SHORT-baseline one. They add weight to BA
        without adding constraint and pull points toward the local cluster,
        diluting the long-baseline observations the strided pair graph exists
        to create. That sits beside the earlier finding that requiring LONG
        baselines for triangulation partners also hurt — this pipeline is
        sensitive to the baseline MIX, and both ends of it.

        It also cost ~600 MB of descriptors held resident through BA, which
        is real money on the device. Removed rather than left default-off: a
        knob nothing reads is the defect this whole audit is about.
        **If anyone tries again, take candidates from FAR cameras that see
        the track, not near ones, and measure ATE — not track length.** Mean
        track length went up in both attempts while the model got worse, so
        it is not the metric to optimize.
      - ~~**`boot_h_over_e_max = 0.45`**~~ — **this entry was wrong.** The
        behaviour ships and always did: `EstimateRelativePose` computes the
        homography inlier count beside the essential one and sets
        `planar_ambiguous` when `inliers_h > 0.85 * inliers_e`. That is the
        plan's `inl_H/inl_E >= 0.45` in the other algebraic form — the plan
        quotes ORB-SLAM's `S_H/(S_H+S_E) > 0.45`, which rearranges to
        `S_H > 0.818 * S_E`. And the live bootstrap does something better
        than the plan's blind reject: a planar pair is VALIDATED against
        LiDAR depth (the wrong branch of the conjugate-plane ambiguity
        scatters the depth ratios, the right one clusters), so a genuinely
        planar start is usable rather than refused. The threshold is
        hardcoded rather than configurable, which is the only real defect
        here and a much smaller one than "never built".

- [ ] **THE performance problem: the final solve registers under half the
      images at the density a real capture actually has.** The user's figure
      is ~200 images per room, so two rooms is ~400 over a 116 m walk — 29 cm
      apart. Measured there:

      | | registered | points | rmse | ATE |
      |---|---|---|---|---|
      | lap angled along the wall | 132/400 (33%) | 32.2k | 0.54 px | 0.205 m |
      | lap hugging + facing across | **189/400 (47%)** | 39.3k | 0.57 px | **0.133 m** |

      What registers is geometrically sound, so this is connectivity, not
      accuracy. Two structural causes found, both being measured now:
      1. **The pair graph has no appearance retrieval.** S3 is sequential
         ±8 plus fixed index strides {12,20,32,52,84} plus live-pose
         proximity — and the live pass produces nothing usable at store
         cadence, so in practice it is index-only. The capture walk revisits
         places hundreds of frames apart (the lap passes a wall that an
         orbit views 200 frames later; the walk closes at frame 400 against
         frame 0) and no index stride reaches that far. The graph is a chain
         and every break splits off a component with literally zero shared
         points. `final_bow_top_k` exists in the config and nothing reads
         it: the S2 BoW stage from the plan was never built.
      2. **`final_sift_max_frames = 250` silently drops the quality preset
         to ORB above that.** 400 frames is over it, so the measurements
         above are ORB — the more viewpoint-sensitive detector, on a walk
         designed to view surfaces from many angles. The cap is about
         transient descriptor memory and was set before anyone knew a room
         is 200 images.
      **Answered: (2) was the whole registration story, and a third problem
      was hiding behind it.** With SIFT the solve registers 400/400 at 0.35 px
      — and reported 1.26 m ATE, which was neither warp nor scale (model/truth
      distance ratio 0.999 even 200 frames apart) but **21 cameras registered
      on evidence too thin to hold a pose**: a median of 119 observations
      against the model's 899, at 1.67 px against 0.33. The other 379 were at
      **2.2 cm**. Those 21 also broke levelling, by pulling the camera-centre
      plane the floor search uses for its up-direction.
      The solve now unregisters a camera with under 25% of the median
      observation count AND over 2x the median per-image error — both,
      because either alone is an honest state — and puts the frame back in
      the pool for PnP to try again next round. End to end on the same
      session:

      | | registered | points | rmse | ATE |
      |---|---|---|---|---|
      | ORB (shipped) | 189/400 | 39.3k | 0.57 px | 0.205 m |
      | SIFT | 400/400 | 79.6k | 0.35 px | 1.256 m |
      | **SIFT + drop weak cameras** | **378/400** | **83.3k** | **0.32 px** | **0.063 m** |

      **Answered: it is (2), and it is the whole thing.** Forced SIFT on the
      same 400 frames registers **400/400** with 79.6k points at 0.35 px,
      against 189/400 and 39.3k at 0.57 px with ORB. So the index-only pair
      graph is adequate after all — when the descriptor can match across
      viewpoints — and (1) is a real gap but not the one costing anything
      today. The cap is now a memory budget in bytes rather than a frame
      count, and the app fills it from `os_proc_available_memory()`.
      (The exhaustive-pairing run was abandoned once SIFT answered it; 80k
      pairs of ORB was going to take another half hour to tell us something
      we no longer needed.)

- [x] ~~**Levelling picks the wrong plane on a dense model.**~~ **It was not
      levelling.** 21 of 400 cameras were metres out of place while the other
      379 sat at 2.2 cm; the floor search takes its up-direction from a plane
      fit to the camera centres, and cameras scattered off the walk are not
      in that plane, so the true floor was rejected. Fixed at the source —
      see the entry above. Original note kept below for the diagnosis trail.

- [x] ~~**Levelling picks the wrong plane on a dense model.**~~ Found by the
      SIFT run above, and it is the reason that run's ATE is 1.26 m despite
      400/400 registered at 0.35 px: the reconstruction is excellent and the
      levelling put it somewhere wrong. Floor search found a plane with
      **413 inliers** and placed the cameras 1.96 m above it with 0.84 m
      spread; the ORB run on the same session found **10,786 inliers**, 1.48 m
      above, spread 0.011 m — and ground truth eye height is 1.5 m. More
      points made the search worse, which points at the candidate ranking
      rather than at the data. Note the fixture has no `--floor-calib`, so
      this is the inference path; a user who does the floor step would not
      hit it. Next: log every candidate plane the search scored and see what
      beat the floor.

- [ ] **The final solve fragments on the capture walk, and density does not
      fix it.** (Largely superseded by the SIFT finding above — re-measure
      before spending anything on it.) On the two-room circle-and-orbit walk the batch solve
      registers a third to a half of the images and leaves the rest in
      components that "share no alignable structure (0 candidate points)":

      | frames | spacing | registered | ATE |
      |---|---|---|---|
      | 340 | 0.35 m | 199/340 | 0.051 m |
      | 340 (lap stood back) | 0.31 m | 119/340 | 0.339 m |
      | 480 | 0.22 m | 156/480 | 1.340 m |

      Same scene single-room, 140 frames: **140/140 at 0.0017 m**. So it is
      not the solver being weak in general and it is not sampling density —
      it is this path. The obvious suspects, in order: the pair graph is
      sequential ±8 plus BoW top-10, and on this walk ±8 frames spans a
      whole orbit arc, so the sequential half contributes little that the
      BoW half did not; and consecutive MOVES (orbit a table, then orbit a
      cabinet 5 m away) genuinely share little, so the graph's connectivity
      rests on the approach legs, which are short.
      Worth knowing before acting: a device stores at 5-10 cm, 3-6x denser
      than any of these fixtures, so this may be a fixture artefact — but
      480 frames getting WORSE says it is not simply that. Next step is
      cheap: dump the pair graph's connected components for the 340-frame
      case and see whether the missing edges are ones the BoW stage should
      have found.
      Until then the split-export fixture uses the single-room scene
      (scripts/validate_colmap.py says why), because a gate that swings from
      0.05 m to 1.34 m on unrelated changes is not a gate.

- [x] ~~**The store gate is too dense for the capture flow: ~900 frames per
      room where the budget is 200-500.**~~ **Done: scaled by scene depth.**
      A flat distance cannot be right for both an orbit at 2 m and a wall at
      6 m — what sets the overlap between neighbouring stored images is the
      ratio of baseline to subject distance, not the baseline. The gate is now
      `max(5 cm, 4% of median scene depth)`, taken from the points of the most
      recent keyframe (not the depth image, which goes blank past the LiDAR's
      5 m on exactly the wide shots this is for; not the whole map, which
      would count the far room through a doorway).

      | | flat 5 cm | 4% of depth |
      |---|---|---|
      | room A | 1,096 | 382 |
      | room B | 948 | 423 |
      | whole walk | 2,044 | 805 |

      (All simulated, and all superseded — see the measured table below.)

      In practice 24 cm down a wall at 6 m, 11 cm around a table at 2.7 m,
      5 cm wherever the phone is close enough for the floor to bind. A test
      asserts a room lands in 200-500 rather than merely "not absurd".

      **The live replay came back and the simulation was 36% low.** Fed
      through the actual engine (`bs_replay --live` on the two-room walk,
      3370 frames), the gate keeps **1267**, not the 805 simulated — about
      **633 per room, above the 200-500 band**, and over the app's 500-frame
      cap for a two-room session. The gap is the depth estimate, and the
      difference is real rather than an artefact: the simulation used the
      true distance to the first surface along the optical axis, the engine
      uses the median depth of the tracked points in the last keyframe, and
      tracked points sit on near, textured surfaces. The engine's number is
      the more honest one for overlap — what fills the frame is what has to
      overlap — but it is smaller, so the spacing is smaller and the count is
      larger. Re-measuring at `store_translation_depth_frac = 0.06` now;
      even that is ~14 cm at the observed 2.3 m median, which is still ~95%
      frame overlap, so there is room to move without hurting the solve.
      **Do not trust a gate count that was not measured through the
      engine.**

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

- [ ] **A tiled room still loses live tracking half the time, and that is
      not the matcher's fault.** `--repetitive 0.30` (new) reproduces a
      kitchen/bathroom: exactly-repeating tile over the floor and textured
      walls. The final solve is fixed — see the ratio change — but the LIVE
      pass drops from 95.0% tracked to **48.3%** and ends LOST, and
      tightening the ratio makes it worse rather than better (40.0% at 0.7,
      33.3% at 0.6), because the guided 15 px search plus the local map has
      already disambiguated and the ratio test is only discarding good
      matches. Where it does track it is accurate to 6 mm, so this is an
      honest refusal, not a wrong answer — but a user in a tiled bathroom
      gets a TRACKING LOST pill half the time.
      Worth trying, in rough order of promise: raise the ORB feature budget
      when the inlier ratio sags (more candidates in the window); use the
      LiDAR depth to reject candidates at the wrong distance, which
      disambiguates tiles perfectly and costs nothing since the depth is
      already there; or accept it and tell the user what is happening, since
      "point at something with more variety" is real advice.

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

- **ARKit poses for the LIVE VIEW ONLY** — keeps the map view and recovery
  arrow alive through a tracking loss, which is when they are most needed and
  least available. Display layer only: never through the C ABI, never in RAW,
  and above all never in `live/poses.jsonl`, which looks like a log but is
  read by the final solve as pose init and as the position source for rescan
  supersession. Needs an engine-to-ARKit frame alignment that goes stale
  during the loss it exists to cover, so it must fade rather than lie. See
  ARCHITECTURE.md for the guardrails.

- **ARKit capture backend (depth + confidence only, poses still image-first).**
  Engine seam is DONE and tested (`DepthImage.confidence`). What remains is
  iOS-side and unverifiable without hardware — see the table in
  ARCHITECTURE.md "Why not ARKit". Build it as a swappable backend beside the
  AVFoundation one, never as a replacement, so the field test can A/B them on
  the same room. Blocked on answering: does
  `configurableCaptureDeviceForPrimaryCamera` really allow AE/AWB lock; is
  `.sceneDepth` unsmoothed; is there a distortion LUT.

- **Restore user region names when a project is reopened.** Renames are now
  PERSISTED to session.json (they used to die with the session), but nothing
  reads them back into the engine. Naive restore-by-id is wrong: region ids
  are derived fresh from the covisibility graph each session, so region 1 in
  one capture is not region 1 in the next. Needs region identity that is
  stable across captures — probably spatial (the readiness bounds now in
  report.json would do it) rather than ordinal.


- Exposure/white-balance normalization across frames before the final solve.
- Per-session depth-vs-triangulation affine correction (risk 6 in the plan).
- Surface the flagged images in the app, not just in `report.json` — a
  "these 12 frames are hurting your model" list on the Processing or Export
  screen, with the option to exclude and re-solve.

---

## Log

Newest first. One line per session: what changed, what it measured.

- **The capture walk is now circle-then-orbit, and the doorway is a real
  wall.** Two changes that only make sense together. The test scene's divider
  was a zero-thickness plane with a hole in it, so the "doorway" had no
  jambs and no soffit — nothing inside the opening to see from an angle,
  nothing for LiDAR to return, and no reason to orbit it. It is now a 16 cm
  partition with two faces, two jambs and a soffit. And `CaptureTrajectory`
  (was `WalkthroughTrajectory`) replaces a hand-typed 33 m loop with a plan
  computed from the layout: circle each room at the largest inset the
  furniture allows, then orbit every large object in it, then orbit the
  doorway from that room's side, then cross. Orbit radius is half the room's
  short dimension, cut back to what is walkable. Hand-typed waypoints encode
  one furniture arrangement and walk through the sofa when it moves; this
  re-plans, and `TwoRoomWalkable()` lets tests assert no pose ends up inside
  a solid.

  | | value |
  |---|---|
  | path per room | 62 m / 57 m (was 33 m for both) |
  | orbit radius asked / achieved | 3.0 m / 1.6–2.6 m |
  | bearings per object | 4–11 of 12 sectors; doorway 11 |
  | worst single frame | 1.33 deg = exactly the declared pan cap |
  | poses inside a solid | 0 of 3645 |

  Three harness defects fell out of it, all of the "obeys its own limit and
  is still impossible" family (detail in ARCHITECTURE.md): look targets
  blended as POINTS sweep through the camera; the same for the smoothing
  window; and the view rate limiter slewed along the great circle, so a
  ~180 deg pan went **over the pole** — 11 deg/frame of pose while every
  consecutive pair of look directions differed by exactly the 1.33 deg cap.
  Now blended as bearing/range/height and limited in yaw and pitch, with a
  35 deg pitch clamp because the roll-locked camera frame degenerates near
  vertical.

  **The lap looks 45 deg along the wall, not square at it.** First version
  pointed the camera straight out while walking half a metre off the wall,
  and the measurement was total rather than gradual: every orbit in the
  capture tracked and **every lap tracked at zero** — room B's whole 26 m lap
  lost, 748 consecutive frames, plus the first 12 m of room A before the
  capture could relocalize into the scaffold at all. Where it did track,
  median error was 1.3 cm. A camera 50 cm from a flat wall is not a tracking
  problem to be solved, it is a picture nobody wants.

  | capture pass, seed 7, 118 m | square at the wall | 45 deg along it |
  |---|---|---|
  | tracked | 54.6% | **62.4%** |
  | rigid ATE | 0.118 m / 0.27 deg | **0.029 m** / 0.41 deg |
  | anchored ATE | 5.290 m / 42.2 deg | **0.067 m** / 0.72 deg |
  | keyframes / points | 626 / 29.1k | **675 / 32.5k** |
  | ends | LOST | **TRACKING** |

  The scout circuit, unchanged in shape, went 85.9% -> **99.4%** tracked at
  0.021 m rigid, purely from the yaw/pitch rate limiter replacing the
  great-circle slew. The split fixture went 99 -> **199 of 340** registered
  and 26.7k -> **54.9k** points from the same one-line change to where the
  lap looks.

  **CI split fixture re-sampled, 110 -> 340 frames**, because --two-room is
  now 118 m rather than 33 m and a frame count over a fixed path is a
  spacing. 110 frames = 94 cm apart lands 0.74 m from ground truth; 180
  fragments into six components and lands 7.0 m out; 340 (35 cm, matching
  what the fixture always had) comes in at **0.051 m / 0.31 deg, 199 of 340
  registered, 5 parts**. The script now gates on that ATE: every other
  assertion in the split section passes just as happily on a model metres out
  of place. Costs ~7 min of CI.

  **And a real engine bug fell out of it: "at least two live poses" is not a
  usable initialization.** A live pass that bootstrapped and immediately lost
  tracking left 2 posed frames out of 340, with the scale lock 2.7x out; the
  final solve seeded itself on them and finished at **6/340 registered,
  1,573 points, 7.0 m ATE**. Discarding them and bootstrapping from image
  geometry — which is what `build_component` is for, and it picks a
  well-conditioned pair on purpose — gave 99/340 and 26,734 points at 0.054
  m. S6 now requires the hint to cover `max(8, 5%)` of the session and drops
  it wholesale otherwise (leaving even one live pose in place mixes two world
  gauges). New metric `live_poses_used` records the decision, because the
  outcome does not: a 14-frame scene recovers from a bad seed either way.

  The **store gate was too dense** for it — 944 frames for one room against a
  200-500 budget — and is now scaled by scene depth, which puts a room at
  382/423 in simulation — and 1267 for the whole walk when finally measured
  through the engine, which is what moved it to 6% of depth and a 15 deg
  rotation term. See the entry above.

- **End of capture is no longer a dead end.** It said "Saved:
  session_20260814-142230_a3f2c1" with a Done button — a filename and an
  exit, at the one moment the user is still standing in the room and could
  fix it. Now: frames against the 200-500 band, a verdict, and for a THIN
  capture the primary action is "Walk it again" rather than "Done". That
  timing is the whole value — a thin capture looks fine at the time and only
  shows as holes hours later, when fixing it means a return trip. Walking it
  again starts a NEW capture in the same project (RAW is write-once, a
  finalized session is closed) so it inherits the world frame and adds the
  coverage the first pass missed; it is not a rescan, nothing is replaced.

- **Two fields in session.json that lied.** `keyframe_ids` was empty in
  every device session ever captured — the app polls `bs_live_status`, which
  DRAINS the engine's storage directives, and threw them away. Now drained
  and recorded at finalize (the keyframe decision comes after the frame is
  written, so meta.json cannot carry it). And `regions` was a hardcoded
  `[{id: 1, name: "Room 1"}]` claiming a region existed before anything was
  mapped, while actual user renames went only to the engine's in-memory map
  and died with the session — though FORMATS.md documents them as
  persisted. Renames now reach session.json; reading them BACK is a separate
  problem, recorded in Ideas.

- **Readiness recomputed from the FINAL solve** — the last named gap.
  PatchGrid scores a LiveMap, so the solve assembles one from its own
  frames and tracks rather than growing a second scoring implementation to
  drift from the first. Clean scene: **61% overall, 189 patches, 1 region**,
  sub-scores [73.6 geom, 96.4 pose, 32.7 texture, 84.5 lidar, 72.9 view].
  The two readiness numbers answer different questions and both earn their
  place: live asks "is this room worth more time now", the report asks "is
  this worth GPU hours".

  It also improved the rescan flow, which had been free-text because no room
  bounds existed after a solve. "Redo a room" now lists real rooms **worst
  score first** with their weakest axis — the stored bounds inform the
  CHOICE, the walked volume still determines the EFFECT.

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

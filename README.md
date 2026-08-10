# BetterSplats

A native iPhone app for **Gaussian-splat-ready photogrammetry capture** on
LiDAR iPhones. Walk through an environment, get live per-room "splat
readiness" feedback, then produce a globally optimized **COLMAP sparse
reconstruction** (`cameras.txt`, `images.txt`, `points3D.txt`, `images/`)
ready for splat training — all on the phone, fully offline.

**Image-first by design**: camera poses come only from multi-view image
geometry (SfM with global bundle adjustment). LiDAR is an independent depth
sensor with its own confidence model, used to validate geometry, anchor
textureless surfaces and suppress floaters — never to track the camera.
ARKit is not used.

## Status

The full reconstruction engine works end-to-end on Linux CI (76 unit tests
plus a synthetic ground-truth pipeline): capture format + immutable RAW
storage, two-view geometry, LiDAR confidence + adaptive fusion, live
incremental SfM (95% tracked / 3 mm ATE on the synthetic room), the
splat-readiness scoring system, and the final global reconstruction —
60/60 frames registered at 2.2 mm ATE, COLMAP export loads in pycolmap
(track length 4.6, 0.41 px reprojection), fused `dense.ply`, and a
verified byte-identical RAW layer after processing.

The iOS app (capture, live guidance, readiness dashboard, 3D map,
on-device reconstruction + share-sheet export) builds via the `ios-app`
workflow; grab the IPA artifact and see `docs/SIDELOADING.md`. On-device
field testing is the current frontier — expect tuning commits as real
captures come back through the replay tool.

## Build matrix (no Mac required)

| What | Where | How |
|---|---|---|
| Engine + tests | Linux (any) | `scripts/ci_linux.sh deps && scripts/ci_linux.sh all` |
| Engine + tests | GitHub Actions | `core-linux` workflow, every push |
| iPhone app (unsigned IPA) | GitHub Actions macOS runner | `ios-app` workflow → `BetterSplats-ipa` artifact |
| Install on iPhone | your PC + cable | see `docs/SIDELOADING.md` (Sideloadly/AltStore, free Apple ID) |

Local engine development:

```sh
sudo scripts/ci_linux.sh deps       # apt: Eigen, OpenCV, Ceres, gtest, ninja
cmake --preset linux-rel
cmake --build --preset linux-rel
ctest --preset linux-rel
./build/linux-rel/tools/replay/bs_replay --selftest
```

## Repository layout

```
core/         portable C++17 reconstruction engine + tests
tools/        bs_synth (synthetic sessions), bs_replay (Linux session replay)
ios/          SwiftUI app; Xcode project generated from project.yml (XcodeGen)
scripts/      CI entry points, iOS dependency builder, IPA packaging
third_party/  vendored single-file deps (licenses in third_party/README.md)
docs/         ARCHITECTURE.md · FORMATS.md · SIDELOADING.md
```

## Design documents

- `docs/ARCHITECTURE.md` — engine design, data layers, fusion model,
  splat-readiness scoring
- `docs/FORMATS.md` — normative on-disk formats (session, depth codec,
  COLMAP export conventions)
- `docs/SIDELOADING.md` — installing CI-built IPAs with a free Apple ID

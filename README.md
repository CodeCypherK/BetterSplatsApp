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

Early development — M0 (scaffold) complete: portable C++ engine skeleton
with Eigen/OpenCV/Ceres proven on Linux CI and cross-compiled to iOS, plus
a diagnostics app shell. See `docs/ARCHITECTURE.md` for the full design and
the milestone plan.

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

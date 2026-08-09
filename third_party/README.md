# Vendored third-party code

Single-file dependencies vendored directly so the engine builds identically
on Linux and iOS with no package-manager involvement.

| Component | Version | License | Source |
|---|---|---|---|
| `nlohmann/json.hpp` | 3.11.3 | MIT | https://github.com/nlohmann/json (single_include) |
| `lz4/lz4.{h,c}` | 1.10.0 | BSD 2-Clause | https://github.com/lz4/lz4 (lib/) |

Larger dependencies are NOT vendored:

- **Eigen 3.4** — apt on Linux; headers installed into `ios-deps/` by
  `scripts/ios/build_deps.sh` for iOS.
- **OpenCV** — apt on Linux; official prebuilt `opencv2.framework` on iOS.
- **Ceres 2.2** — apt on Linux; built for iOS arm64 by
  `scripts/ios/build_deps.sh` (miniglog, no suitesparse).

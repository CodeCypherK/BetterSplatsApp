#!/usr/bin/env bash
# Builds/collects the third-party binaries needed to cross-compile the engine
# for iOS into ios-deps/:
#   opencv2.framework/             prebuilt official OpenCV (device slice)
#   include/opencv2/               extracted headers for CMake
#   include/eigen3/, share/eigen3/ Eigen headers + CMake config
#   include/ceres/, lib/libceres.a Ceres built for iOS arm64 with miniglog
#
# Idempotent: a marker file derived from versions.lock short-circuits the
# whole script, so a warm CI cache costs ~1 second.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DEPS="$ROOT/ios-deps"
LOCK="$ROOT/scripts/ios/versions.lock"

# shellcheck source=versions.lock
source "$LOCK"

lock_hash="$(shasum -a 256 "$LOCK" | cut -c1-16)"
MARKER="$DEPS/.built-$lock_hash"
if [[ -f "$MARKER" ]]; then
  echo "ios-deps already built for lock $lock_hash"
  exit 0
fi

echo "Building ios-deps (lock $lock_hash)..."
rm -rf "$DEPS"
mkdir -p "$DEPS/src" "$DEPS/lib" "$DEPS/include"

# ---------------------------------------------------------------- 1. OpenCV
echo "--- OpenCV $OPENCV_VERSION (prebuilt xcframework)"
curl -fL --retry 3 --retry-delay 5 -o "$DEPS/src/opencv.zip" "$OPENCV_URL"
actual_sha="$(shasum -a 256 "$DEPS/src/opencv.zip" | cut -d' ' -f1)"
if [[ -n "$OPENCV_SHA256" && "$actual_sha" != "$OPENCV_SHA256" ]]; then
  echo "ERROR: OpenCV archive sha256 mismatch:"
  echo "  expected $OPENCV_SHA256"
  echo "  actual   $actual_sha"
  exit 1
fi
if [[ -z "$OPENCV_SHA256" ]]; then
  echo "WARNING: OPENCV_SHA256 is unpinned. Pin this value in versions.lock:"
  echo "  OPENCV_SHA256=\"$actual_sha\""
fi
unzip -q "$DEPS/src/opencv.zip" -d "$DEPS/src/opencv"

# The release archive layout has changed across versions: some ship
# opencv2.xcframework, some a bare (fat) opencv2.framework. Normalize both
# to $DEPS/opencv2.framework — a device-linkable static framework.
framework=""
xcf="$(find "$DEPS/src/opencv" -type d -name 'opencv2.xcframework' | head -1)"
if [[ -n "$xcf" ]]; then
  framework="$(find "$xcf" -type d -path '*ios-arm64*/opencv2.framework' \
    | grep -v -i simulator | grep -v -i macos | head -1)"
else
  framework="$(find "$DEPS/src/opencv" -type d -name 'opencv2.framework' \
    | grep -v -i simulator | grep -v -i macos | head -1)"
fi
if [[ -z "$framework" ]]; then
  echo "ERROR: no opencv2 framework found in archive; contents:"
  find "$DEPS/src/opencv" -maxdepth 4 | head -50
  exit 1
fi
rm -rf "$DEPS/opencv2.framework"
cp -R "$framework" "$DEPS/opencv2.framework"

rm -rf "$DEPS/include/opencv2"
cp -R "$DEPS/opencv2.framework/Headers" "$DEPS/include/opencv2"
echo "OpenCV -> ios-deps/opencv2.framework (headers in ios-deps/include/opencv2)"

# ----------------------------------------------------------------- 2. Eigen
echo "--- Eigen $EIGEN_TAG (headers + CMake config)"
git clone --quiet --depth 1 --branch "$EIGEN_TAG" "$EIGEN_REPO" "$DEPS/src/eigen"
cmake -S "$DEPS/src/eigen" -B "$DEPS/src/eigen-build" \
  -DCMAKE_INSTALL_PREFIX="$DEPS" \
  -DEIGEN_BUILD_DOC=OFF -DBUILD_TESTING=OFF -DEIGEN_BUILD_PKGCONFIG=OFF \
  >/dev/null
cmake --install "$DEPS/src/eigen-build" >/dev/null
echo "Eigen -> ios-deps/include/eigen3"

# ----------------------------------------------------------------- 3. Ceres
echo "--- Ceres $CERES_TAG (iOS arm64 static, miniglog)"
git clone --quiet --depth 1 --branch "$CERES_TAG" "$CERES_REPO" "$DEPS/src/ceres"
# Ceres's CMakeLists has a legacy iOS block that validates IOS_PLATFORM and
# IOS_DEPLOYMENT_TARGET (variables its old bundled toolchain used to set);
# satisfy it explicitly while modern CMake handles the actual cross-compile.
cmake -S "$DEPS/src/ceres" -B "$DEPS/src/ceres-build" \
  -DCMAKE_SYSTEM_NAME=iOS \
  -DIOS_PLATFORM=OS \
  -DIOS_DEPLOYMENT_TARGET="$IOS_DEPLOYMENT_TARGET" \
  -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_OSX_SYSROOT=iphoneos \
  -DCMAKE_OSX_DEPLOYMENT_TARGET="$IOS_DEPLOYMENT_TARGET" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$DEPS" \
  -DCMAKE_PREFIX_PATH="$DEPS" \
  -DCMAKE_FIND_ROOT_PATH="$DEPS" \
  -DEigen3_DIR="$DEPS/share/eigen3/cmake" \
  -DMINIGLOG=ON \
  -DGFLAGS=OFF \
  -DSUITESPARSE=OFF \
  -DLAPACK=OFF \
  -DEIGENSPARSE=ON \
  -DBUILD_TESTING=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_BENCHMARKS=OFF \
  -DBUILD_SHARED_LIBS=OFF \
  -DBUILD_DOCUMENTATION=OFF
cmake --build "$DEPS/src/ceres-build" -j "$(sysctl -n hw.ncpu 2>/dev/null || nproc)"
cmake --install "$DEPS/src/ceres-build" >/dev/null
echo "Ceres -> ios-deps/lib/libceres.a"

rm -rf "$DEPS/src"
touch "$MARKER"
echo "ios-deps build complete"

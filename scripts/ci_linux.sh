#!/usr/bin/env bash
# Single entry point for the Linux CI pipeline; each stage is also runnable
# locally (e.g. `scripts/ci_linux.sh build`). PRESET selects the CMake
# preset (default linux-rel; the weekly sanitizer job sets linux-asan).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PRESET="${PRESET:-linux-rel}"
STAGE="${1:-all}"

deps() {
  export DEBIAN_FRONTEND=noninteractive
  apt-get update -qq
  apt-get install -y -qq \
    libeigen3-dev libopencv-dev libceres-dev libgtest-dev libgmock-dev \
    ninja-build ccache
}

build() {
  cd "$ROOT"
  if command -v ccache >/dev/null; then
    export CMAKE_CXX_COMPILER_LAUNCHER=ccache
    export CMAKE_C_COMPILER_LAUNCHER=ccache
  fi
  cmake --preset "$PRESET"
  cmake --build --preset "$PRESET"
}

test_stage() {
  cd "$ROOT"
  ctest --preset "$PRESET"
}

validate() {
  cd "$ROOT"
  # Spec invariant: ARKit tracking is forbidden anywhere in the codebase.
  if grep -rn --include='*.swift' --include='*.h' --include='*.cpp' \
       -e 'import ARKit' -e 'ARSession' -e 'ARWorldTracking' \
       ios/ core/ tools/; then
    echo "ERROR: ARKit usage detected — image-first SfM only" >&2
    exit 1
  fi
  # Spec invariant: every EngineConfig field is read by the engine. A knob
  # nothing reads is a documented lie, and this found eight of them.
  python3 scripts/check_config_used.py

  # End-to-end pipeline validation: synth session -> replay live -> final
  # solve -> COLMAP export -> pycolmap assertions + RAW immutability.
  python3 scripts/validate_colmap.py --build-dir "build/$PRESET"
}

case "$STAGE" in
  deps) deps ;;
  build) build ;;
  test) test_stage ;;
  validate) validate ;;
  all) build; test_stage; validate ;;
  *) echo "unknown stage: $STAGE (deps|build|test|validate|all)"; exit 2 ;;
esac

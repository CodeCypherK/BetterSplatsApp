cmake_minimum_required(VERSION 3.24)
project(BetterSplats LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BS_BUILD_TESTS "Build unit/integration tests (host platforms only)" ON)
option(BS_BUILD_TOOLS "Build synth/replay CLI tools (host platforms only)" ON)
option(BS_SANITIZE "Enable ASan + UBSan" OFF)

if(BS_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "iOS")
  # Cross builds resolve Eigen/Ceres/OpenCV headers from the prebuilt deps tree
  # produced by scripts/ios/build_deps.sh (see docs/ARCHITECTURE.md).
  set(BS_IOS_DEPS_DIR "" CACHE PATH "Prefix containing iOS-built Ceres/Eigen and extracted OpenCV headers")
  if(NOT BS_IOS_DEPS_DIR)
    message(FATAL_ERROR "BS_IOS_DEPS_DIR must point at the ios-deps prefix for iOS builds")
  endif()
  list(APPEND CMAKE_PREFIX_PATH "${BS_IOS_DEPS_DIR}")
  # CMAKE_SYSTEM_NAME=iOS activates find-root isolation: host prefix paths
  # are ignored unless also present in the find root path.
  list(APPEND CMAKE_FIND_ROOT_PATH "${BS_IOS_DEPS_DIR}")
endif()

add_subdirectory(core)

if(BS_BUILD_TOOLS)
  add_subdirectory(tools/synth)
  add_subdirectory(tools/replay)
endif()

if(BS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(core/tests)
endif()

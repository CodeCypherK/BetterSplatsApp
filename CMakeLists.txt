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
  if(MSVC)
    message(FATAL_ERROR "BS_SANITIZE is GCC/Clang-only — use the linux-asan preset")
  endif()
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

# The engine is portable C++17 and the Windows port needed no source changes,
# but MSVC differs from the GNU family in three ways that are settled here
# rather than in the sources:
#   _USE_MATH_DEFINES  M_PI is the ONLY non-portable construct in core/ and
#                      tools/ (16 uses). MSVC hides it behind this macro, and
#                      it must be defined before <cmath> — hence a command-line
#                      define rather than a #define in a header, which would
#                      depend on include order.
#   /bigobj            The Ceres cost functors in final_solve.cpp and
#                      live_mapper.cpp expand past MSVC's 65,536-section limit.
#   /utf-8             Sources carry UTF-8 comments; without this MSVC reads
#                      them in the system codepage and warns (C4819) or worse.
# NOMINMAX and WIN32_LEAN_AND_MEAN are defensive: nothing here includes
# windows.h today, but OpenCV and Ceres headers can pull it in transitively,
# and its min/max macros break std::min/std::max and Eigen.
if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES NOMINMAX WIN32_LEAN_AND_MEAN)
  add_compile_options(/bigobj /utf-8)
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

# bscore: the portable reconstruction engine. Static library, identical
# sources on Linux (tests/tools) and iOS (linked into the app).

set(BS_VERSION "0.1.0")

find_package(Eigen3 3.4 REQUIRED)
find_package(Ceres REQUIRED)

add_library(bscore STATIC
  src/calib/lut_fit.cpp
  src/common/config.cpp
  src/engine/api.cpp
  src/engine/engine_impl.cpp
  src/engine/selftest.cpp
  src/fusion/residuals.cpp
  src/fusion/scale.cpp
  src/geometry/triangulation.cpp
  src/geometry/two_view.cpp
  src/lidar/depth_processing.cpp
  src/io/depth_codec.cpp
  src/io/session_schema.cpp
  src/io/session_reader.cpp
  src/io/session_writer.cpp
  src/live/live_map.cpp
  src/live/live_mapper.cpp
  src/live/live_system.cpp
  src/vision/features.cpp
  src/vision/matching.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/lz4/lz4.c
)

# Version string embeds the short git hash when available.
find_package(Git QUIET)
set(BS_GIT_HASH "unknown")
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE BS_GIT_HASH_RAW
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(BS_GIT_HASH_RAW)
    set(BS_GIT_HASH ${BS_GIT_HASH_RAW})
  endif()
endif()
target_compile_definitions(bscore PRIVATE
  BS_VERSION_NUM="${BS_VERSION}"
  BS_GIT_HASH="${BS_GIT_HASH}")

target_include_directories(bscore
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party
)

target_link_libraries(bscore PUBLIC Eigen3::Eigen Ceres::ceres)

if(CMAKE_SYSTEM_NAME STREQUAL "iOS")
  # OpenCV comes from the official prebuilt xcframework; build_deps.sh
  # extracts its headers into ios-deps/include (as opencv2/...). Symbols are
  # resolved when the app links the xcframework.
  target_include_directories(bscore SYSTEM PRIVATE ${BS_IOS_DEPS_DIR}/include)
else()
  find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d flann)
  target_include_directories(bscore SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(bscore PRIVATE ${OpenCV_LIBS})
endif()

target_compile_options(bscore PRIVATE -Wall -Wextra)

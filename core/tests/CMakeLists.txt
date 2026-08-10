find_package(GTest REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs features2d)

add_executable(bs_tests
  test_api.cpp
  test_colmap_export.cpp
  test_component_align.cpp
  test_config.cpp
  test_depth_codec.cpp
  test_final_resume.cpp
  test_fusion.cpp
  test_lidar.cpp
  test_lut_fit.cpp
  test_map_io.cpp
  test_matching.cpp
  test_readiness.cpp
  test_session_io.cpp
  test_synth.cpp
  test_triangulation.cpp
  test_two_view.cpp
)

target_link_libraries(bs_tests PRIVATE bscore synthlib GTest::gtest
  GTest::gtest_main ${OpenCV_LIBS})
target_include_directories(bs_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../src
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party
  ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/synth)
target_include_directories(bs_tests SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})

include(GoogleTest)
gtest_discover_tests(bs_tests DISCOVERY_TIMEOUT 60)

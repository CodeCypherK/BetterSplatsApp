find_package(GTest REQUIRED)

add_executable(bs_tests
  test_api.cpp
  test_config.cpp
)

target_link_libraries(bs_tests PRIVATE bscore GTest::gtest GTest::gtest_main)
target_include_directories(bs_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)

include(GoogleTest)
gtest_discover_tests(bs_tests DISCOVERY_TIMEOUT 60)

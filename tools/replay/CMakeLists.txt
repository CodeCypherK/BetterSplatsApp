add_executable(bs_replay main.cpp)
target_link_libraries(bs_replay PRIVATE bscore)
target_include_directories(bs_replay PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../core/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party)

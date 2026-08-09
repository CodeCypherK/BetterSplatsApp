add_executable(bs_synth main.cpp)
target_link_libraries(bs_synth PRIVATE bscore)
target_include_directories(bs_synth PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../core/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party)

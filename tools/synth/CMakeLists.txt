# synthlib is linked by the CLI and by core/tests (integration tests generate
# sessions in-process).
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_library(synthlib STATIC
  synth_scene.cpp
  generate.cpp
)
target_include_directories(synthlib
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../core/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party
)
target_include_directories(synthlib SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(synthlib PUBLIC bscore PRIVATE ${OpenCV_LIBS})

add_executable(bs_synth main.cpp)
target_link_libraries(bs_synth PRIVATE synthlib)
target_include_directories(bs_synth PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../core/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party)
# --dump-plan reaches into synth_scene.h, which is built on cv::Vec3f.
target_include_directories(bs_synth SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})

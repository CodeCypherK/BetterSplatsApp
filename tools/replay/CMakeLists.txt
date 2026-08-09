find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_executable(bs_replay main.cpp)
target_link_libraries(bs_replay PRIVATE bscore ${OpenCV_LIBS})
target_include_directories(bs_replay PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../core/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party)
target_include_directories(bs_replay SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})

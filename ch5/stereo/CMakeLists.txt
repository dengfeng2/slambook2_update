cmake_minimum_required( VERSION 3.5 )
project(stereoVision)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

add_executable(stereoVision stereoVision.cpp)
target_include_directories(stereoVision PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stereoVision ${OpenCV_LIBRARIES} Eigen3::Eigen ${Pangolin_LIBRARIES})
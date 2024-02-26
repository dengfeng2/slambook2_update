cmake_minimum_required( VERSION 3.5 )
project( ch3_example )

set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(Eigen3 REQUIRED)

add_executable(coordinateTransform coordinateTransform.cpp)
target_link_libraries(coordinateTransform Eigen3::Eigen)

find_package(Pangolin REQUIRED)
add_executable(plotTrajectory plotTrajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
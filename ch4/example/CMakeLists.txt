cmake_minimum_required( VERSION 3.5 )
project(ch4_example)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

add_executable(trajectoryError trajectoryError.cpp)
target_link_libraries(trajectoryError Eigen3::Eigen Sophus::Sophus ${Pangolin_LIBRARIES})
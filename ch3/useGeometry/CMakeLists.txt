cmake_minimum_required( VERSION 3.5 )
project( geometry )

find_package(Eigen3 REQUIRED)

add_executable(eigenGeometry eigenGeometry.cpp)
target_link_libraries(eigenGeometry Eigen3::Eigen)

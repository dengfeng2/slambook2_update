cmake_minimum_required( VERSION 3.5 )
project( visualizeGeometry )

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(GLEW REQUIRED)

add_executable(visualizeGeometry visualizeGeometry.cpp)
target_link_libraries(visualizeGeometry Eigen3::Eigen)
target_link_libraries(visualizeGeometry ${Pangolin_LIBRARIES} GLEW::GLEW)
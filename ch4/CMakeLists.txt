cmake_minimum_required( VERSION 3.5 )
project(ch4)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus Eigen3::Eigen Sophus::Sophus)

add_subdirectory(example)
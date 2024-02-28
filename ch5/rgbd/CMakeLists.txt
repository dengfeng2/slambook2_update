cmake_minimum_required( VERSION 3.5 )
project(joinMap)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)

add_executable(joinMap joinMap.cpp)
target_include_directories(joinMap PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(joinMap Eigen3::Eigen fmt::fmt ${OpenCV_LIBRARIES} Sophus::Sophus ${Pangolin_LIBRARIES})
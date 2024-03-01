cmake_minimum_required( VERSION 3.5 )
project(ch7)

find_package(OpenCV REQUIRED)

add_executable(orb_cv orb_cv.cpp)
target_include_directories(orb_cv PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(orb_cv ${OpenCV_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4")
add_executable(orb_self orb_self.cpp)
target_include_directories(orb_self PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(orb_self ${OpenCV_LIBRARIES})
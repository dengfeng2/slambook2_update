cmake_minimum_required( VERSION 3.5 )
project(imageBasics)

find_package(OpenCV REQUIRED)

add_executable(imageBasics imageBasics.cpp)
target_include_directories(imageBasics PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(imageBasics ${OpenCV_LIBRARIES})

add_executable(undistortImage undistortImage.cpp)
target_include_directories(undistortImage PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(undistortImage ${OpenCV_LIBRARIES})
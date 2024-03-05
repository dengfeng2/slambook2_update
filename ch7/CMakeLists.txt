cmake_minimum_required( VERSION 3.5 )
project(ch7)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

add_executable(orb_cv orb_cv.cpp)
target_include_directories(orb_cv PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(orb_cv ${OpenCV_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++17 -O2 ${SSE_FLAGS} -msse4")
add_executable(orb_self orb_self.cpp)
target_include_directories(orb_self PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(orb_self ${OpenCV_LIBRARIES})

add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)
target_include_directories(pose_estimation_2d2d PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBRARIES})

add_executable(triangulation triangulation.cpp)
target_include_directories(triangulation PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(triangulation ${OpenCV_LIBRARIES})

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
if (POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE LEGACY)
endif()
find_package(g2o REQUIRED)
add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)
target_include_directories(pose_estimation_3d2d PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(pose_estimation_3d2d ${OpenCV_LIBRARIES} Eigen3::Eigen Sophus::Sophus g2o::core g2o::solver_eigen g2o::solver_dense)

add_executable(pose_estimation_3d3d pose_estimation_3d3d.cpp)
target_include_directories(pose_estimation_3d3d PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(pose_estimation_3d3d ${OpenCV_LIBRARIES} Eigen3::Eigen Sophus::Sophus g2o::core g2o::solver_eigen g2o::solver_dense)

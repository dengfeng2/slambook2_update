cmake_minimum_required( VERSION 3.5 )
project(ch6)

find_package(Eigen3 REQUIRED)
add_executable(gaussNewton gaussNewton.cpp)
target_link_libraries(gaussNewton Eigen3::Eigen)

find_package(Ceres REQUIRED)
add_executable(ceresCurveFitting ceresCurveFitting.cpp)
target_link_libraries(ceresCurveFitting Ceres::ceres)

if (POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE LEGACY)
endif()
find_package(g2o REQUIRED)
add_executable(g2oCurveFitting g2oCurveFitting.cpp)
target_link_libraries(g2oCurveFitting g2o::core g2o::solver_eigen g2o::solver_dense)
cmake_minimum_required(VERSION 3.5)  # 声明要求的cmake最低版本

project(HelloSLAM)  # 声明一个cmake工程

set(CMAKE_BUILD_TYPE "Debug")  # 设置编译模式

add_executable(HelloSLAM helloSLAM.cpp)  # 添加一个可执行程序

add_library(hello libHelloSLAM.cpp)  # 添加hello库
add_library(hello_shared SHARED libHelloSLAM.cpp)  # 共享库

add_executable(useHello useHello.cpp)  # 添加可执行程序调用hello库中的函数
target_include_directories(useHello PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(useHello hello_shared)  # 将库文件链接到可执行程序上
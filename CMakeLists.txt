cmake_minimum_required(VERSION 3.22)
project(rmcs_auto_aim_v2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Wall -Wextra -Wpedantic -O3)

include(FetchContent)

FetchContent_Declare(
    tinympc
    GIT_REPOSITORY https://github.com/Alliance-Algorithm/TinyMPC.git
    GIT_TAG main
    SOURCE_SUBDIR src/tinympc
)
FetchContent_MakeAvailable(tinympc)
if(TARGET tinympcstatic AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(tinympcstatic PRIVATE -w)
endif()

## 依赖查找
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)

## ROS2 相关依赖
find_package(type_description_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

find_package(rmcs_msgs REQUIRED)
find_package(rmcs_executor REQUIRED)
find_package(rmcs_description REQUIRED)
find_package(fast_tf REQUIRED)
find_package(hikcamera REQUIRED)


## 代码资源搜索
file(GLOB_RECURSE AUTO_AIM_UTILITY
    CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/utility/*.cpp
    ${PROJECT_SOURCE_DIR}/src/utility/*.cc
)
file(GLOB_RECURSE AUTO_AIM_KERNEL
    CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/kernel/*.cpp
    ${PROJECT_SOURCE_DIR}/src/kernel/*.cc
)
file(GLOB_RECURSE AUTO_AIM_MODULE
    CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/module/*.cpp
    ${PROJECT_SOURCE_DIR}/src/module/*.cc
)

## 基础设施: utility
add_library(
    ${PROJECT_NAME}_utility SHARED
    ${AUTO_AIM_UTILITY}
)
target_include_directories(
    ${PROJECT_NAME}_utility
    PUBLIC
        ${PROJECT_SOURCE_DIR}/src/
        ${rmcs_msgs_INCLUDE_DIRS}
    PRIVATE
        ${visualization_msgs_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${tf2_ros_INCLUDE_DIRS}
)
target_link_libraries(
    ${PROJECT_NAME}_utility
    PUBLIC
        yaml-cpp::yaml-cpp
        ament_index_cpp::ament_index_cpp
        rclcpp::rclcpp
        ${OpenCV_LIBS}
        ${rmcs_msgs_LIBRARIES}
    PRIVATE
        ${visualization_msgs_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${tf2_ros_LIBRARIES}
)

## 核心构建: kernel and module
add_library(
    ${PROJECT_NAME}_kernel SHARED
    ${AUTO_AIM_KERNEL}
)
target_include_directories(
    ${PROJECT_NAME}_kernel
    PUBLIC
        ${PROJECT_SOURCE_DIR}/src/
    PRIVATE
        ${hikcamera_INCLUDE_DIRS}
)
target_link_libraries(
    ${PROJECT_NAME}_kernel
    PUBLIC
        ${PROJECT_NAME}_utility
)

add_library(
    ${PROJECT_NAME}_module SHARED
    ${AUTO_AIM_MODULE}
)
target_include_directories(
    ${PROJECT_NAME}_module
    PUBLIC
        ${PROJECT_SOURCE_DIR}/src/
    PRIVATE
        ${hikcamera_INCLUDE_DIRS}
)
target_link_libraries(
    ${PROJECT_NAME}_module
    PUBLIC
        ${PROJECT_NAME}_utility
        openvino::runtime
    PRIVATE
        tinympcstatic
        ${hikcamera_LIBRARIES}
)

## 组件构建: component
add_library(
    ${PROJECT_NAME}_component SHARED
    ${PROJECT_SOURCE_DIR}/src/component.cpp
)
target_include_directories(
    ${PROJECT_NAME}_component
    PRIVATE
        ${rmcs_description_INCLUDE_DIRS}
        ${rmcs_executor_INCLUDE_DIRS}
)
target_link_libraries(
    ${PROJECT_NAME}_component
    ${PROJECT_NAME}_kernel
    ${PROJECT_NAME}_module
    ${rmcs_executor_LIBRARIES}
)

## 安装配置
install(
    TARGETS
        ${PROJECT_NAME}_component
        ${PROJECT_NAME}_kernel
        ${PROJECT_NAME}_module
        ${PROJECT_NAME}_utility
    DESTINATION
        lib/
)
install(
    DIRECTORY
        config/
        models/
    DESTINATION
        share/${PROJECT_NAME}
)

## plugins for rmcs_executor
pluginlib_export_plugin_description_file(
    rmcs_executor plugins.xml
)

find_package(ament_cmake REQUIRED)
if(BUILD_TESTING)
    add_subdirectory(test)
endif()

ament_package()

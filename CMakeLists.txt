cmake_minimum_required(VERSION 3.22)
project(rmcs_auto_aim_v2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Wall -Wextra -Wpedantic)

## 依赖查找
find_package(OpenCV 4.5 REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(Ceres REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rmcs_executor REQUIRED)
find_package(rmcs_msgs REQUIRED)
find_package(rmcs_description REQUIRED)
find_package(fast_tf REQUIRED)

## 目录包含
include_directories(
	${PROJECT_SOURCE_DIR}/src/
	${rmcs_executor_INCLUDE_DIRS}
	${rmcs_msgs_INCLUDE_DIRS}
	${rmcs_description_INCLUDE_DIRS}
	${fast_tf_INCLUDE_DIRS}
)

## 代码资源搜索
file(GLOB_RECURSE AUTO_AIM_SOURCE
	CONFIGURE_DEPENDS
	${PROJECT_SOURCE_DIR}/src/kernel/*.cpp
	${PROJECT_SOURCE_DIR}/src/kernel/*.cc
)

## 构建目标
add_library(
	${PROJECT_NAME}_library SHARED
	${AUTO_AIM_SOURCE}
)
target_link_libraries(
	${PROJECT_NAME}_library
	${HikCameraSDK_LIB}
	${OpenCV_LIBS}
	openvino::runtime
	Ceres::ceres
)

add_library(
	${PROJECT_NAME}_component SHARED
	${PROJECT_SOURCE_DIR}/src/component.cpp
)
target_link_libraries(
	${PROJECT_NAME}_component
	${PROJECT_NAME}_library
	rclcpp::rclcpp
	${rmcs_executor_LIBRARIES}
)

add_executable(
    ${PROJECT_NAME}_runtime
	${PROJECT_SOURCE_DIR}/src/runtime.cpp
)
target_link_libraries(
	${PROJECT_NAME}_runtime
	${PROJECT_NAME}_library
	rclcpp::rclcpp
)

## 安装配置
install(
	DIRECTORY models/
	DESTINATION share/${PROJECT_NAME}/models
)
install(
	TARGETS
	${PROJECT_NAME}_runtime
	${PROJECT_NAME}_component
	DESTINATION
	lib/${PROJECT_NAME}
)

find_package(pluginlib REQUIRED)
pluginlib_export_plugin_description_file(
	rmcs_executor plugins.xml
)

find_package(ament_cmake REQUIRED)
ament_package()
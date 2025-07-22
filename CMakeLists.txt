cmake_minimum_required(VERSION 3.22)
project(rmcs_auto_aim_v2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE "Release")

set(EXECUTABLE_NAME ${PROJECT_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## 依赖查找
find_package(ament_cmake  	REQUIRED)

find_package(OpenCV 4.5   	REQUIRED)
find_package(OpenVINO     	REQUIRED)
find_package(Ceres        	REQUIRED)

find_package(PkgConfig 		REQUIRED)
pkg_check_modules(FFMPEG  	REQUIRED
	libavformat
	libavcodec
	libavutil
	libswscale
)

## 目录包含
include_directories(
	${PROJECT_SOURCE_DIR}/src
	${FFMPEG_INCLUDE_DIRS}
)

## 代码资源搜索
file(GLOB_RECURSE AUTO_AIM_SOURCE
	CONFIGURE_DEPENDS
	${PROJECT_SOURCE_DIR}/src/*.cpp
	${PROJECT_SOURCE_DIR}/src/*.cc
)

## 构建目标
find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()
ament_auto_add_library(
	${EXECUTABLE_NAME} SHARED
	${AUTO_AIM_SOURCE}
)

target_link_libraries(
	${EXECUTABLE_NAME}
	${OpenCV_LIBS}
	${HikCameraSDK_LIB}
	${FFMPEG_LIBRARIES}
	openvino::runtime
	Ceres::ceres
	-lpthread
)

## 安装配置
pluginlib_export_plugin_description_file(
	rmcs_executor plugins.xml
)
install(
    DIRECTORY models/
    DESTINATION share/${PROJECT_NAME}/models
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# comment the line when a copyright and license is added to all source files
	set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# comment the line when this package is in a git repo and when
	# a copyright and license is added to all source files
	set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.22)
project(rmcs_auto_aim_v2_tools)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(RMCS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(RMCS_SRC_DIR "${RMCS_SOURCE_DIR}/src")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(OpenCV 4.5 REQUIRED)

find_package(hikcamera QUIET)

set(HIKCAMERA_AVAILABLE OFF)
if(hikcamera_FOUND)
    set(HIKCAMERA_AVAILABLE ON)
elseif(DEFINED hikcamera_LIBRARIES)
    set(HIKCAMERA_AVAILABLE ON)
endif()

include_directories(
    ${RMCS_SRC_DIR}
    ${hikcamera_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

set(TOOL_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Visualization
add_executable(
    visualization
    ${TOOL_DIR}/visualization.cpp
    ${RMCS_SRC_DIR}/utility/rclcpp/visual/armor.cpp
    ${RMCS_SRC_DIR}/utility/rclcpp/visual/posture.cpp
    ${RMCS_SRC_DIR}/utility/panic.cpp
    ${RMCS_SRC_DIR}/utility/math/solve_armors.cpp
    ${RMCS_SRC_DIR}/utility/rclcpp/node.cpp
)
target_link_libraries(
    visualization
    rclcpp::rclcpp
    ${visualization_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
)

if(HIKCAMERA_AVAILABLE)
    # Hikcamera
    add_executable(
        hikcamera
        ${TOOL_DIR}/hikcamera.cpp
    )
    target_link_libraries(
        hikcamera
        ${OpenCV_LIBS}
        ${hikcamera_LIBRARIES}
    )

    # RTP Streaming
    add_executable(
        streaming
        ${TOOL_DIR}/streaming.cpp
        ${RMCS_SRC_DIR}/module/capturer/hikcamera.cpp
        ${RMCS_SRC_DIR}/module/debug/visualization/stream_session.cpp
        ${RMCS_SRC_DIR}/module/debug/visualization/stream_context.cpp
        ${RMCS_SRC_DIR}/utility/image/image.cpp
        ${RMCS_SRC_DIR}/utility/rclcpp/node.cpp
    )
    target_link_libraries(
        streaming
        rclcpp::rclcpp
        ${OpenCV_LIBS}
        ${hikcamera_LIBRARIES}
    )
else()
    message(STATUS "hikcamera 未检测到，跳过相关构建")
endif()

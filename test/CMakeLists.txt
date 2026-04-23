cmake_minimum_required(VERSION 3.22)
project(rmcs_auto_aim_v2_tests)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(RMCS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(RMCS_SRC_DIR "${RMCS_SOURCE_DIR}/src")

if(NOT TARGET tinympcstatic)
    add_subdirectory(
        ${RMCS_SOURCE_DIR}/third_party/tinympc/src/tinympc
        ${CMAKE_CURRENT_BINARY_DIR}/tinympc
    )
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(tinympcstatic PRIVATE -w)
    endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(OpenCV 4.5 REQUIRED)

include_directories(
    ${RMCS_SRC_DIR}
    ${RMCS_SOURCE_DIR}/third_party/tinympc/src
    ${hikcamera_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Duck
ament_add_gtest(
    test_duck_type
    ${TEST_DIR}/duck_type.cpp
)

# Pipeline
ament_add_gtest(
    test_pipeline
    ${TEST_DIR}/pipeline.cpp
)

# Serializable
ament_add_gtest(
    test_serializable
    ${TEST_DIR}/serializable.cpp
)
target_link_libraries(
    test_serializable
    rclcpp::rclcpp
    yaml-cpp::yaml-cpp
)

# Interprocess
ament_add_gtest(
    test_interprocess
    ${TEST_DIR}/interprocess.cpp
)

# Model Infer
ament_add_gtest(
    test_model_infer
    ${TEST_DIR}/model_infer.cpp
    ${RMCS_SRC_DIR}/module/identifier/armor_detection.cpp
    ${RMCS_SRC_DIR}/utility/image/image.cpp
)
target_link_libraries(
    test_model_infer
    yaml-cpp::yaml-cpp
    openvino::runtime
    ${OpenCV_LIBRARIES}
)

# Device Id
ament_add_gtest(
    test_device_id
    ${TEST_DIR}/device_id.cpp
)

# Image Recorder
ament_add_gtest(
    test_image_recorder
    ${TEST_DIR}/image_recorder.cpp
    ${RMCS_SRC_DIR}/utility/image/recorder.cpp
)
target_link_libraries(
    test_image_recorder
    ${OpenCV_LIBRARIES}
)

# Solve pnp
ament_add_gtest(
    test_solve_pnp
    ${TEST_DIR}/solve_pnp.cpp
    ${RMCS_SRC_DIR}/utility/math/solve_pnp/pnp_solution.cpp
    ${RMCS_SRC_DIR}/utility/math/solve_pnp/yaw_optimizer.cpp
    ${RMCS_SRC_DIR}/utility/math/camera.cpp
    ${RMCS_SRC_DIR}/module/identifier/armor_detection.cpp
    ${RMCS_SRC_DIR}/utility/image/image.cpp
)
ament_target_dependencies(
    test_solve_pnp
    rclcpp
    visualization_msgs
    geometry_msgs
)
target_link_libraries(
    test_solve_pnp
    ${OpenCV_LIBRARIES}
    openvino::runtime
    yaml-cpp::yaml-cpp
)

# Static TF
ament_add_gtest(
    test_static_tf
    ${TEST_DIR}/static_tf.cpp
    ${RMCS_SRC_DIR}/utility/panic.cpp
)
target_link_libraries(
    test_static_tf
    yaml-cpp::yaml-cpp
)

# Transform Communication
ament_add_gtest(
    test_transform_communication
    ${TEST_DIR}/transform_communication.cpp
)

# Feishu IPC
ament_add_gtest(
    test_feishu
    ${TEST_DIR}/feishu_test.cpp
)


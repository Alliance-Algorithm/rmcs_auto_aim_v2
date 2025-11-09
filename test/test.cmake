find_package(ament_cmake_gtest REQUIRED)

#
# For auto check
#
ament_add_gtest(
    test_duck_type
    test/duck_type.cpp
)
target_link_libraries(
    test_duck_type
    ${PROJECT_NAME}_kernel
    ${PROJECT_NAME}_module
)

ament_add_gtest(
    test_pipeline
    test/pipeline.cpp
)
target_link_libraries(
    test_pipeline
    ${PROJECT_NAME}_module
)

ament_add_gtest(
    test_serializable
    test/serializable.cpp
)
target_link_libraries(
    test_serializable
    ${PROJECT_NAME}_module
    rclcpp::rclcpp
    yaml-cpp::yaml-cpp
)

ament_add_gtest(
    test_interprocess
    test/interprocess.cpp
)

ament_add_gtest(
    test_model_infer
    test/model_infer.cpp
)
target_link_libraries(
    test_model_infer
    yaml-cpp::yaml-cpp
    ${PROJECT_NAME}_module
)

#
# For quick test
#
add_executable(
    example_hikcamera
    test/hikcamera.cpp
)
target_link_libraries(
    example_hikcamera
    ${PROJECT_NAME}_module
)

add_executable(
    example_streaming
    test/streaming.cpp
)
target_link_libraries(
    example_streaming
    rclcpp::rclcpp
    ${PROJECT_NAME}_module
)

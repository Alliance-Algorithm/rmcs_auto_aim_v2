find_package(ament_cmake_gtest REQUIRED)
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
    test_streaming
    test/streaming.cpp
)
target_link_libraries(
    test_streaming
    ${PROJECT_NAME}_kernel
    ${PROJECT_NAME}_module
)
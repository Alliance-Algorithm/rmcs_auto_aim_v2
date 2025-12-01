#include "utility/math/solve_pnp.hpp"
#include "utility/math/linear.hpp"
#include "utility/math/point.hpp"

#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <gtest/gtest.h>

using namespace rmcs;
using namespace rmcs::util;
using Eigen::Vector2d;
using Eigen::Vector3d;

// 辅助函数：创建测试用的相机内参
PnpSolution::Input create_test_input(double focal_length = 800.0, double cx = 320.0,
    double cy = 240.0, const std::array<double, 5>& distort = { 0.0, 0.0, 0.0, 0.0, 0.0 }) {
    PnpSolution::Input input;
    input.camera_matrix = { {
        { focal_length, 0.0, cx },
        { 0.0, focal_length, cy },
        { 0.0, 0.0, 1.0 },
    } };
    input.distort_coeff = distort;
    return input;
}

// 辅助函数：创建小装甲板的 3D 点
std::array<Point3d, 4> create_small_armor_shape() {
    constexpr double ARMOR_WIDTH  = 0.135; // 135mm
    constexpr double ARMOR_HEIGHT = 0.056; // 56mm

    const auto eigen_points = std::array {
        Point3d { 0.0, ARMOR_WIDTH / 2.0, ARMOR_HEIGHT / 2.0 },   // 右上
        Point3d { 0.0, -ARMOR_WIDTH / 2.0, ARMOR_HEIGHT / 2.0 },  // 右下
        Point3d { 0.0, -ARMOR_WIDTH / 2.0, -ARMOR_HEIGHT / 2.0 }, // 左下
        Point3d { 0.0, ARMOR_WIDTH / 2.0, -ARMOR_HEIGHT / 2.0 }   // 左上
    };

    auto points = std::array<Point3d, 4>();
    for (size_t i = 0; i < 4; i++) {
        points[i] = Point3d(eigen_points[i]);
    }
    return points;
}

// 辅助函数：创建大装甲板的 3D 点
std::array<Point3d, 4> create_big_armor_shape() {
    constexpr double BIG_ARMOR_WIDTH = 0.230; // 230mm
    constexpr double ARMOR_HEIGHT    = 0.056; // 56mm

    const auto eigen_points = std::array {
        Point3d { 0.0, BIG_ARMOR_WIDTH / 2.0, ARMOR_HEIGHT / 2.0 },
        Point3d { 0.0, -BIG_ARMOR_WIDTH / 2.0, ARMOR_HEIGHT / 2.0 },
        Point3d { 0.0, -BIG_ARMOR_WIDTH / 2.0, -ARMOR_HEIGHT / 2.0 },
        Point3d { 0.0, BIG_ARMOR_WIDTH / 2.0, -ARMOR_HEIGHT / 2.0 },
    };

    auto points = std::array<Point3d, 4>();
    for (size_t i = 0; i < 4; i++) {
        points[i] = Point3d(eigen_points[i]);
    }
    return points;
}

// 辅助函数：从 Eigen::Vector2d 创建 Point2d 数组
std::array<Point2d, 4> create_armor_detection(const std::array<Vector2d, 4>& eigen_points) {
    auto points = std::array<Point2d, 4>();
    for (size_t i = 0; i < 4; i++) {
        points[i] = Point2d(eigen_points[i]);
    }
    return points;
}

// 辅助函数：验证四元数是否归一化
bool is_quaternion_normalized(const Orientation& q, double tolerance = 0.01) {
    const auto norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return std::abs(norm - 1.0) < tolerance;
}

// 辅助函数：计算两点之间的距离
double distance(const Translation& a, const Translation& b) {
    const auto dx = a.x - b.x;
    const auto dy = a.y - b.y;
    const auto dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// ========== 测试用例 ==========

class PnpSolverTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 每个测试前的设置
    }

    void TearDown() override {
        // 每个测试后的清理
    }
};

TEST_F(PnpSolverTest, BasicSmallArmor) {
    auto solution = PnpSolution {};

    solution.input             = create_test_input();
    solution.input.armor_shape = create_small_armor_shape();

    // 使用 Eigen::Vector2d 创建 2D 检测点
    const auto eigen_detection = std::array {
        Vector2d { 350.0, 220.0 },
        Vector2d { 290.0, 220.0 },
        Vector2d { 290.0, 260.0 },
        Vector2d { 350.0, 260.0 },
    };

    solution.input.armor_detection = create_armor_detection(eigen_detection);

    // 执行求解
    EXPECT_NO_THROW(solution.solve());

    // 验证结果
    EXPECT_GT(solution.result.translation.z, 0.0) //
        << "Translation z should be positive (in front of camera)";

    EXPECT_TRUE(is_quaternion_normalized(solution.result.orientation)) //
        << "Quaternion should be normalized";

    const auto trans_norm = std::sqrt(solution.result.translation.x * solution.result.translation.x
        + solution.result.translation.y * solution.result.translation.y
        + solution.result.translation.z * solution.result.translation.z);

    EXPECT_GT(trans_norm, 0.0) << "Translation should not be zero";
}

TEST_F(PnpSolverTest, BigArmor) {
    auto solution = PnpSolution {};

    solution.input             = create_test_input();
    solution.input.armor_shape = create_big_armor_shape();

    // 使用 Eigen::Vector2d 创建 2D 检测点
    const auto eigen_detection = std::array {
        Vector2d { 380.0, 200.0 },
        Vector2d { 260.0, 200.0 },
        Vector2d { 260.0, 280.0 },
        Vector2d { 380.0, 280.0 },
    };

    solution.input.armor_detection = create_armor_detection(eigen_detection);

    // 执行求解
    EXPECT_NO_THROW(solution.solve());

    // 验证结果
    EXPECT_GT(solution.result.translation.z, 0.0) //
        << "Translation z should be positive (in front of camera)";

    EXPECT_TRUE(is_quaternion_normalized(solution.result.orientation)) //
        << "Quaternion should be normalized";
}

TEST_F(PnpSolverTest, DifferentDistances) {
    constexpr double FOCAL_LENGTH = 800.0;
    constexpr double ARMOR_WIDTH  = 0.135;

    const auto test_distances = std::vector<double> { 1.0, 2.0, 3.0, 5.0 };

    for (const auto expected_distance : test_distances) {
        auto solution = PnpSolution {};

        solution.input             = create_test_input(FOCAL_LENGTH);
        solution.input.armor_shape = create_small_armor_shape();

        // 根据距离计算图像中的大小
        const auto pixel_size = (FOCAL_LENGTH * ARMOR_WIDTH) / expected_distance;
        const auto center     = Vector2d { 320.0, 240.0 };

        const auto eigen_detection = std::array<Vector2d, 4> {
            center + Vector2d { pixel_size / 2, -pixel_size * 0.2 },
            center + Vector2d { -pixel_size / 2, -pixel_size * 0.2 },
            center + Vector2d { -pixel_size / 2, pixel_size * 0.2 },
            center + Vector2d { pixel_size / 2, pixel_size * 0.2 },
        };

        solution.input.armor_detection = create_armor_detection(eigen_detection);

        // 执行求解
        EXPECT_NO_THROW(solution.solve());

        // 验证结果
        const auto distance_error = std::abs(solution.result.translation.z - expected_distance);

        EXPECT_LT(distance_error, expected_distance * 0.5) //
            << "Distance error too large for expected distance " << expected_distance;

        EXPECT_TRUE(is_quaternion_normalized(solution.result.orientation)) //
            << "Quaternion should be normalized";
    }
}

TEST_F(PnpSolverTest, EdgeCaseSmallDetection) {
    auto solution = PnpSolution {};

    solution.input             = create_test_input();
    solution.input.armor_shape = create_small_armor_shape();

    // 使用 Eigen::Vector2d 创建 2D 检测点（小检测框，远距离）
    const auto eigen_detection = std::array {
        Vector2d { 322.0, 238.0 },
        Vector2d { 318.0, 238.0 },
        Vector2d { 318.0, 242.0 },
        Vector2d { 322.0, 242.0 },
    };

    solution.input.armor_detection = create_armor_detection(eigen_detection);

    // 执行求解
    EXPECT_NO_THROW(solution.solve());

    // 验证结果
    EXPECT_GT(solution.result.translation.z, 0.0) //
        << "Translation z should be positive (in front of camera)";

    EXPECT_TRUE(is_quaternion_normalized(solution.result.orientation)) //
        << "Quaternion should be normalized";
}

TEST_F(PnpSolverTest, WithDistortion) {
    auto solution = PnpSolution {};

    solution.input = create_test_input(800.0, 320.0, 240.0, { 0.1, -0.2, 0.0, 0.0, 0.0 });
    solution.input.armor_shape = create_small_armor_shape();

    // 使用 Eigen::Vector2d 创建 2D 检测点
    const auto eigen_detection = std::array {
        Vector2d { 350.0, 220.0 },
        Vector2d { 290.0, 220.0 },
        Vector2d { 290.0, 260.0 },
        Vector2d { 350.0, 260.0 },
    };

    solution.input.armor_detection = create_armor_detection(eigen_detection);

    // 执行求解
    EXPECT_NO_THROW(solution.solve());

    // 验证结果
    EXPECT_GT(solution.result.translation.z, 0.0) //
        << "Translation z should be positive (in front of camera)";

    EXPECT_TRUE(is_quaternion_normalized(solution.result.orientation)) //
        << "Quaternion should be normalized";
}

TEST_F(PnpSolverTest, QuaternionValidity) {
    auto solution = PnpSolution {};

    solution.input             = create_test_input();
    solution.input.armor_shape = create_small_armor_shape();

    // 使用 Eigen::Vector2d 创建 2D 检测点
    const auto eigen_detection = std::array {
        Vector2d { 350.0, 220.0 },
        Vector2d { 290.0, 220.0 },
        Vector2d { 290.0, 260.0 },
        Vector2d { 350.0, 260.0 },
    };

    solution.input.armor_detection = create_armor_detection(eigen_detection);

    // 执行求解
    solution.solve();

    // 验证四元数的有效性
    const auto& q = solution.result.orientation;

    EXPECT_TRUE(std::isfinite(q.x)) << "Quaternion x should be finite";
    EXPECT_TRUE(std::isfinite(q.y)) << "Quaternion y should be finite";
    EXPECT_TRUE(std::isfinite(q.z)) << "Quaternion z should be finite";
    EXPECT_TRUE(std::isfinite(q.w)) << "Quaternion w should be finite";

    const auto norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

    EXPECT_NEAR(norm, 1.0, 0.01) << "Quaternion should be normalized";
}

TEST_F(PnpSolverTest, TranslationValidity) {
    auto solution = PnpSolution {};

    solution.input             = create_test_input();
    solution.input.armor_shape = create_small_armor_shape();

    // 使用 Eigen::Vector2d 创建 2D 检测点
    const auto eigen_detection = std::array {
        Vector2d { 350.0, 220.0 },
        Vector2d { 290.0, 220.0 },
        Vector2d { 290.0, 260.0 },
        Vector2d { 350.0, 260.0 },
    };

    solution.input.armor_detection = create_armor_detection(eigen_detection);

    // 执行求解
    solution.solve();

    // 验证平移向量的有效性
    const auto& t = solution.result.translation;

    EXPECT_TRUE(std::isfinite(t.x)) << "Translation x should be finite";
    EXPECT_TRUE(std::isfinite(t.y)) << "Translation y should be finite";
    EXPECT_TRUE(std::isfinite(t.z)) << "Translation z should be finite";

    EXPECT_GT(t.z, 0.0) << "Translation z should be positive";
}

TEST_F(PnpSolverTest, DifferentFocalLengths) {
    const auto focal_lengths = std::vector<double> { 400.0, 800.0, 1200.0, 1600.0 };

    for (const auto focal : focal_lengths) {
        auto solution = PnpSolution {};

        solution.input             = create_test_input(focal);
        solution.input.armor_shape = create_small_armor_shape();

        const auto scale  = focal / 800.0;
        const auto center = Vector2d { 320.0, 240.0 };
        const auto offset = Vector2d { 30.0, 20.0 };

        const auto eigen_detection = std::array<Vector2d, 4> {
            center + Vector2d { offset.x() * scale, -offset.y() * scale },
            center + Vector2d { -offset.x() * scale, -offset.y() * scale },
            center + Vector2d { -offset.x() * scale, offset.y() * scale },
            center + Vector2d { offset.x() * scale, offset.y() * scale },
        };

        solution.input.armor_detection = create_armor_detection(eigen_detection);

        // 执行求解
        EXPECT_NO_THROW(solution.solve());

        // 验证结果
        EXPECT_GT(solution.result.translation.z, 0.0) //
            << "Translation z should be positive (in front of camera)";

        EXPECT_TRUE(is_quaternion_normalized(solution.result.orientation)) //
            << "Quaternion should be normalized";
    }
}

TEST_F(PnpSolverTest, Consistency) {
    auto solution1 = PnpSolution {};
    auto solution2 = PnpSolution {};

    solution1.input             = create_test_input();
    solution1.input.armor_shape = create_small_armor_shape();

    // 使用 Eigen::Vector2d 创建 2D 检测点
    const auto eigen_detection = std::array {
        Vector2d { 350.0, 220.0 },
        Vector2d { 290.0, 220.0 },
        Vector2d { 290.0, 260.0 },
        Vector2d { 350.0, 260.0 },
    };

    solution1.input.armor_detection = create_armor_detection(eigen_detection);
    solution2.input                 = solution1.input; // 相同输入

    // 执行求解
    solution1.solve();
    solution2.solve();

    // 验证一致性
    const auto trans_diff = distance(solution1.result.translation, solution2.result.translation);

    EXPECT_LT(trans_diff, 0.001) << "Results should be consistent";

    const auto quat_diff =
        std::sqrt(std::pow(solution1.result.orientation.x - solution2.result.orientation.x, 2)
            + std::pow(solution1.result.orientation.y - solution2.result.orientation.y, 2)
            + std::pow(solution1.result.orientation.z - solution2.result.orientation.z, 2)
            + std::pow(solution1.result.orientation.w - solution2.result.orientation.w, 2));

    EXPECT_TRUE(quat_diff < 0.001 || quat_diff > 1.9) //
        << "Quaternions should be consistent";
}

TEST_F(PnpSolverTest, Performance) {
    auto solution = PnpSolution {};

    solution.input             = create_test_input();
    solution.input.armor_shape = create_small_armor_shape();

    // 使用 Eigen::Vector2d 创建 2D 检测点
    const auto eigen_detection = std::array {
        Vector2d { 350.0, 220.0 },
        Vector2d { 290.0, 220.0 },
        Vector2d { 290.0, 260.0 },
        Vector2d { 350.0, 260.0 },
    };

    solution.input.armor_detection = create_armor_detection(eigen_detection);

    constexpr int iterations = 1000;
    const auto start         = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < iterations; i++) {
        solution.solve();
    }

    const auto end      = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    const auto avg_time_us = static_cast<double>(duration.count()) / iterations;

    std::cout << "Average solve time: " << avg_time_us << " microseconds\n";

    EXPECT_LT(avg_time_us, 1000.0) << "Solve should be fast enough";
}

// 主函数
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

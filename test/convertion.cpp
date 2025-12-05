#include <eigen3/Eigen/Dense>

#include "utility/math/conversion.hpp"
#include <gtest/gtest.h>

using namespace rmcs::util;

namespace {

template <typename MatA, typename MatB>
void expect_matrix_near(const MatA& a, const MatB& b, double eps = 1e-9) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            EXPECT_NEAR(a(r, c), b(r, c), eps);
        }
    }
}

} // namespace

TEST(Conversion, BasisTransformIdentity) {
    Eigen::Vector3d ex { 1, 0, 0 };
    Eigen::Vector3d ey { 0, 1, 0 };
    Eigen::Vector3d ez { 0, 0, 1 };

    auto T = make_basis_transform(ex, ey, ez, ex, ey, ez);

    expect_matrix_near(T, Eigen::Matrix3d::Identity());
}

TEST(Conversion, CvOpticalToRosMatrix) {
    const auto T = make_cv_optical_to_ros_camera_link();

    Eigen::Matrix3d expected;

    // clang-format off
    expected <<
        0 , 0 , 1,
        -1, 0 , 0,        
        0 , -1, 0;
    // clang-format on

    expect_matrix_near(T, expected);
}

TEST(Conversion, CvToRosTransformRt) {
    Eigen::Matrix3d R_cv = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_cv { 1., 2., 3. };

    const auto [R_ros, t_ros] = cv_optical_to_ros_camera_link(R_cv, t_cv);

    Eigen::Matrix3d T;

    // clang-format off
    T <<
        0 , 0 , 1,
        -1, 0 , 0,
        0 , -1, 0;
    // clang-format on

    expect_matrix_near(R_ros, T);

    Eigen::Vector3d expected_t = T * t_cv;
    EXPECT_DOUBLE_EQ(t_ros.x(), expected_t.x());
    EXPECT_DOUBLE_EQ(t_ros.y(), expected_t.y());
    EXPECT_DOUBLE_EQ(t_ros.z(), expected_t.z());
}

TEST(Conversion, BasisTransformRotate90Z) {
    // 源系与目标系：目标系绕 Z 轴 +90 度
    Eigen::Vector3d ex { 1, 0, 0 };
    Eigen::Vector3d ey { 0, 1, 0 };
    Eigen::Vector3d ez { 0, 0, 1 };

    Eigen::Vector3d ex_rot { 0, 1, 0 };
    Eigen::Vector3d ey_rot { -1, 0, 0 };
    Eigen::Vector3d ez_rot { 0, 0, 1 };

    auto T = make_basis_transform(ex, ey, ez, ex_rot, ey_rot, ez_rot);

    Eigen::Matrix3d expected, actual;
    // clang-format off
    actual <<
        0 , -1, 0,
        1 ,  0, 0,
        0 ,  0, 1;
    // clang-format on
    expected = actual.transpose();
    expect_matrix_near(T, expected);
}

TEST(Conversion, CvToRosWithRotation) {
    // 在 cv 系下绕 Z 轴 90 度，转换后应组合到 T 中
    const auto T = make_cv_optical_to_ros_camera_link();

    Eigen::AngleAxisd aa_cv(M_PI / 2.0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R_cv = aa_cv.toRotationMatrix();
    Eigen::Vector3d t_cv { 0.5, -0.25, 2.0 };

    const auto [R_ros, t_ros] = cv_optical_to_ros_camera_link(R_cv, t_cv);

    Eigen::Matrix3d expected_R = T * R_cv;
    expect_matrix_near(R_ros, expected_R);

    Eigen::Vector3d expected_t = T * t_cv;
    EXPECT_NEAR(t_ros.x(), expected_t.x(), 1e-12);
    EXPECT_NEAR(t_ros.y(), expected_t.y(), 1e-12);
    EXPECT_NEAR(t_ros.z(), expected_t.z(), 1e-12);
}

TEST(Conversion, CvToRosWithNonIdentityBasis) {
    // 非标准源/目标基的组合测试
    Eigen::Vector3d src_x { 0, 1, 0 };  // x 指向原来的 y
    Eigen::Vector3d src_y { -1, 0, 0 }; // y 指向 -x
    Eigen::Vector3d src_z { 0, 0, 1 };

    Eigen::Vector3d dst_x { 0, 0, 1 }; // x -> 原来的 z
    Eigen::Vector3d dst_y { 1, 0, 0 }; // y -> 原来的 x
    Eigen::Vector3d dst_z { 0, 1, 0 }; // z -> 原来的 y

    auto T = make_basis_transform(src_x, src_y, src_z, dst_x, dst_y, dst_z);

    Eigen::Matrix3d expected;
    // 手工计算：dst^T * src
    expected << 0, 0, 1, 0, -1, 0, 1, 0, 0;

    expect_matrix_near(T, expected);
}

TEST(Conversion, BasisVectorMapping) {
    const auto T = make_cv_optical_to_ros_camera_link();

    const Eigen::Vector3d ex_cv { 1, 0, 0 };
    const Eigen::Vector3d ey_cv { 0, 1, 0 };
    const Eigen::Vector3d ez_cv { 0, 0, 1 };

    auto ex_ros = T * ex_cv; // 应为 (0,-1,0)
    auto ey_ros = T * ey_cv; // 应为 (0,0,-1)
    auto ez_ros = T * ez_cv; // 应为 (1,0,0)

    EXPECT_NEAR(ex_ros.x(), 0.0, 1e-12);
    EXPECT_NEAR(ex_ros.y(), -1.0, 1e-12);
    EXPECT_NEAR(ex_ros.z(), 0.0, 1e-12);

    EXPECT_NEAR(ey_ros.x(), 0.0, 1e-12);
    EXPECT_NEAR(ey_ros.y(), 0.0, 1e-12);
    EXPECT_NEAR(ey_ros.z(), -1.0, 1e-12);

    EXPECT_NEAR(ez_ros.x(), 1.0, 1e-12);
    EXPECT_NEAR(ez_ros.y(), 0.0, 1e-12);
    EXPECT_NEAR(ez_ros.z(), 0.0, 1e-12);
}

TEST(Conversion, ArbitraryVectorTransform) {
    Eigen::Vector3d v_cv { 1., 2., 3. };
    auto expected = Eigen::Vector3d { 3., -1., -2. };

    auto [_, t_ros] = cv_optical_to_ros_camera_link(Eigen::Matrix3d::Identity(), v_cv);
    EXPECT_NEAR(t_ros.x(), expected.x(), 1e-12);
    EXPECT_NEAR(t_ros.y(), expected.y(), 1e-12);
    EXPECT_NEAR(t_ros.z(), expected.z(), 1e-12);

    // TODO:manual calulate;
}

TEST(Conversion, InverseTransform) {
    const auto T              = make_cv_optical_to_ros_camera_link();
    Eigen::Matrix3d T_inverse = T.inverse();

    // 1. 验证逆矩阵是否等于转置 (验证纯旋转特性)
    expect_matrix_near(T_inverse, T.transpose());

    // 2. 验证 T 乘以 T^-1 是否等于单位矩阵
    Eigen::Matrix3d Identity = T * T_inverse;
    expect_matrix_near(Identity, Eigen::Matrix3d::Identity());
}

TEST(Conversion, CompositeRotation) {
    const auto T = make_cv_optical_to_ros_camera_link();

    // R1: 绕 cv-x 轴 45°
    Eigen::AngleAxisd aa_x(M_PI / 4.0, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d R1 = aa_x.toRotationMatrix();

    // R2: 绕 cv-y 轴 30°
    Eigen::AngleAxisd aa_y(M_PI / 6.0, Eigen::Vector3d::UnitY());
    Eigen::Matrix3d R2 = aa_y.toRotationMatrix();

    // 1. 源系 (CV) 复合旋转
    Eigen::Matrix3d R_cv_comp = R2 * R1;

    // 2. 期望的目标系复合结果 (T * (R2 * R1))
    Eigen::Matrix3d R_ros_expected = T * R_cv_comp;

    // 3. 实际计算
    auto [R_ros_actual, t_ros_acturl] =
        cv_optical_to_ros_camera_link(R_cv_comp, Eigen::Vector3d::Zero());

    // 验证复合结果是否正确
    expect_matrix_near(R_ros_actual, R_ros_expected);
}

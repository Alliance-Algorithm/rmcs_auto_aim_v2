#include <algorithm> // for std::clamp, std::replace
#include <cstdlib>   // for std::getenv
#include <filesystem>
#include <gtest/gtest.h>
#include <iomanip>  // for std::setprecision
#include <iostream> // for structured output
#include <string>
#include <string_view>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include "module/identifier/model.hpp"
#include "utility/image/image.details.hpp"
#include "utility/math/point.hpp"
#include "utility/math/solve_pnp/pnp_solution.hpp"
#include "utility/robot/armor.hpp"

using namespace rmcs::util;
using namespace rmcs;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

// --- 资源路径 ---
static std::filesystem::path asset_root() {
    if (const char* env = std::getenv("TEST_ASSETS_ROOT"); env && *env) {
        return std::filesystem::path { env };
    }

    const char* default_path = "/tmp/auto_aim";

    return std::filesystem::path { default_path };
}

static std::filesystem::path asset_path(std::string_view filename) {
    return asset_root() / filename;
}

// --- 测试数据 ---
struct PnpTestCase {
    std::string filename;
    double expected_distance_m; // 预期的目标距离（米）
    double expected_angle_deg;  // 预期的偏航角（度，0 或 45）
};

// 所有本地测试数据
const std::vector<PnpTestCase> kPnpTestCases = {
    { "blue-0.5m.jpg", 0.5, 0.0 },
    { "blue-0.5m-45degree.jpg", 0.5, 45.0 },
    { "blue-1.0m.jpg", 1.0, 0.0 },
    { "blue-1.0m-45degree.jpg", 1.0, 45.0 },
    { "blue-2.0m.jpg", 2.0, 0.0 },
    { "blue-2.0m-45degree.jpg", 2.0, 45.0 },
    { "blue-3.0m.jpg", 3.0, 0.0 },
    { "blue-3.0m-45degree.jpg", 3.0, 45.0 },
};

// --- 资源读取辅助函数 ---
// --- 核心辅助函数：相机/装甲板参数 ---
// 辅助函数：创建测试用的相机内参
PnpSolution::Input create_test_input(double fx = 1.722231837421459e+03,
    double fy = 1.724876404292754e+03, double cx = 7.013056440882832e+02,
    double cy = 5.645821718351237e+02, double k1 = -0.064232403853946,
    double k2 = -0.087667493884102, double k3 = 0.792381808294582) {

    auto distort_coeff = std::array<double, 5> { k1, k2, 0, 0, k3 };
    PnpSolution::Input input {};
    input.camera_matrix = { {
        { fx, 0.0, cx },
        { 0.0, fy, cy },
        { 0.0, 0.0, 1.0 },
    } };
    input.distort_coeff = distort_coeff;

    return input;
}

// 辅助函数：使用 OpenCV 坐标系定义的装甲板 3D 点
// [Top Left, Top Right, Bottom Right, Bottom Left]
std::array<Point3d, 4> create_small_armor_shape() { return rmcs::kSmallArmorShapeOpenCV; }

std::array<Point2d, 4> infer_armor_detection_from_file(std::string_view filename) {
    using namespace rmcs::identifier;

    constexpr auto config_yaml = R"(
        model_location: "assets/yolov5.xml"
        infer_device: "AUTO"
        use_roi_segment: false
        use_corner_correction: false
        roi_rows: 640
        roi_cols: 640
        input_rows: 640
        input_cols: 640
        min_confidence: 0.8
        score_threshold: 0.7
        nms_threshold: 0.3
    )";

    auto net  = OpenVinoNet {};
    auto yaml = YAML::Load(config_yaml);

    const auto location    = std::filesystem::path { __FILE__ }.parent_path();
    auto model_location    = location / "../models/yolov5.xml";
    yaml["model_location"] = model_location.string();

    auto cfg_result = net.configure(yaml);
    if (!cfg_result.has_value()) {
        throw std::runtime_error("Failed to configure OpenVinoNet: " + cfg_result.error());
    }

    const auto full_path = asset_path(filename);
    auto cv_mat          = cv::imread(full_path.string(), cv::IMREAD_COLOR);
    if (cv_mat.empty()) {
        throw std::runtime_error("Failed to read image: " + full_path.string());
    }

    auto image          = rmcs::Image {};
    image.details().mat = cv_mat;

    auto infer_result = net.sync_infer(image);
    if (!infer_result.has_value()) {
        throw std::runtime_error("OpenVino inference failed: " + infer_result.error());
    }
    const auto& armors = infer_result.value();
    if (armors.empty()) {
        throw std::runtime_error("No armor detected from image: " + full_path.string());
    }

    const auto& armor = armors.front();
    //  [Top Left, Top Right, Bottom Right, Bottom Left] 与 3D 坐标定义一致
    return std::array<Point2d, 4> {
        Point2d { armor.tl() }, // 0
        Point2d { armor.tr() }, // 1
        Point2d { armor.br() }, // 2
        Point2d { armor.bl() }, // 3
    };
}

// --- PnP 结果处理辅助函数 ---

/**
 * @brief 将弧度值规范化到 [-PI/2, PI/2] 范围内 (对应于 [-90度, 90度])。
 * @return 规范化后的角度，范围在 [-PI/2, PI/2] 之间 (弧度)。
 */
double normalize_angle_90(double angle_rad) {
    constexpr double PI      = M_PI;
    constexpr double HALF_PI = PI / 2.0;

    // 1. 将角度限制在 [-PI, PI] 范围内
    double normalized = std::fmod(angle_rad + PI, 2.0 * PI);
    if (normalized < 0) {
        normalized += 2.0 * PI;
    }
    normalized -= PI;

    // 2. 将角度从 [-PI, PI] 映射到 [-PI/2, PI/2] (180度对称处理)
    if (normalized > HALF_PI) {
        normalized = PI - normalized;
    } else if (normalized < -HALF_PI) {
        normalized = -PI - normalized;
    }

    return std::clamp(normalized, -HALF_PI, HALF_PI);
}

// 四元数转 ZYX 欧拉角 (Yaw, Pitch, Roll)，单位：弧度
static Eigen::Vector3d quaternion_to_euler_rad(const Orientation& q) {
    Quaterniond quat(q.w, q.x, q.y, q.z);                            // Eigen 构造函数期望 (w,x,y,z)
    const auto euler = quat.toRotationMatrix().eulerAngles(2, 1, 0); // yaw(Z), pitch(Y), roll(X)
    return { euler[0], euler[1], euler[2] };
}

// --- 参数化测试类 ---
class PnpSolverParameterizedTest : public ::testing::TestWithParam<PnpTestCase> {
protected:
    PnpSolution solution;
    PnpTestCase test_case;
    double actual_distance;
    double folded_yaw_deg;
    double distance_error;
    double yaw_error;
    const double max_allowed_distance_error_ratio = 0.08; // 8%
    const double max_allowed_yaw_error_deg        = 15.0; // 15 degrees

    void SetUp() override {
        test_case                  = GetParam();
        solution.input             = create_test_input();
        solution.input.armor_shape = create_small_armor_shape();

        const auto detection           = infer_armor_detection_from_file(test_case.filename);
        solution.input.armor_detection = detection;

        std::cerr << std::fixed << std::setprecision(1);
        std::cerr << "[2D_POINTS] | FILE: " << test_case.filename << " | TL(" << detection[0].x
                  << "," << detection[0].y << ")"
                  << " TR(" << detection[1].x << "," << detection[1].y << ")"
                  << " BR(" << detection[2].x << "," << detection[2].y << ")"
                  << " BL(" << detection[3].x << "," << detection[3].y << ")" << std::endl;
        std::cerr << std::defaultfloat; // 恢复默认浮点格式

        SCOPED_TRACE(test_case.filename);

        EXPECT_NO_THROW(solution.solve())
            << "Pnp solve failed for file: " << asset_path(test_case.filename);

        // --- 解算结果处理 ---
        const auto& result = solution.result;
        actual_distance    = result.translation.x;
        distance_error     = std::abs(actual_distance - test_case.expected_distance_m);

        const auto euler_rad        = quaternion_to_euler_rad(result.orientation);
        const double actual_yaw_rad = euler_rad[0];

        const double folded_yaw_rad = normalize_angle_90(actual_yaw_rad);
        folded_yaw_deg              = folded_yaw_rad * 180.0 / M_PI;
        yaw_error                   = std::abs(folded_yaw_deg - test_case.expected_angle_deg);
    }

    // 打印结构化报告
    void PrintStructuredReport() const {
        std::cerr << std::fixed << std::setprecision(4);
        std::cerr << "[TEST_REPORT] |" << std::left << std::setw(50) << test_case.filename << "|";

        // 距离信息
        std::cerr << " DISTANCE: " << std::setw(6) << actual_distance << "m (Exp: " << std::setw(4)
                  << test_case.expected_distance_m << "m) | Error: " << std::setw(6)
                  << distance_error << "m | Status: ";
        if (distance_error < test_case.expected_distance_m * max_allowed_distance_error_ratio) {
            std::cerr << "PASS |";
        } else {
            std::cerr << "FAIL |";
        }

        // 角度信息
        std::cerr << " YAW: " << std::setw(6) << folded_yaw_deg << "deg (Exp: " << std::setw(4)
                  << test_case.expected_angle_deg << "deg) | Error: " << std::setw(6) << yaw_error
                  << "deg | Status: ";
        if (yaw_error < max_allowed_yaw_error_deg) {
            std::cerr << "PASS |" << std::endl;
        } else {
            std::cerr << "FAIL |" << std::endl;
        }
    }
};

// 核心测试：距离和角度精度
TEST_P(PnpSolverParameterizedTest, DistanceAndAngleAccuracy) {
    this->PrintStructuredReport();

    // 1. 距离断言 (8% 误差)
    const double max_dist_error =
        GetParam().expected_distance_m * this->max_allowed_distance_error_ratio;

    EXPECT_LT(this->distance_error, max_dist_error)
        << "Distance error (" << this->distance_error << "m) exceeded " << max_dist_error
        << "m for " << GetParam().filename;

    // 2. 角度断言 (15度误差)
    EXPECT_LT(this->yaw_error, this->max_allowed_yaw_error_deg)
        << "Yaw error (" << this->yaw_error << "deg) exceeded " << this->max_allowed_yaw_error_deg
        << "deg for " << GetParam().filename;
}

// 注册参数化测试
INSTANTIATE_TEST_SUITE_P(AllImageTests, PnpSolverParameterizedTest,
    ::testing::ValuesIn(kPnpTestCases), [](const ::testing::TestParamInfo<PnpTestCase>& info) {
        std::string name = info.param.filename;
        std::replace(name.begin(), name.end(), '.', '_');
        std::replace(name.begin(), name.end(), '-', '_');
        return name;
    });

// 主函数
int main(int argc, char** argv) {
    std::cout << "\n--- Starting PnP Accuracy Tests ---\n";
    std::cout << "[TEST_REPORT] | URL (Simplified Name)                                | DISTANCE: "
                 "Actual (Exp) | Error | Status | YAW: Actual (Exp) | Error | Status |\n";

    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();

    std::cout << "--- PnP Accuracy Tests Finished ---\n";
    return result;
}

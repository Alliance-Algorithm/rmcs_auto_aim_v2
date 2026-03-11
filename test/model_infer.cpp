#include "assets_manager.hpp"

#include "module/identifier/armor_detection.hpp"
#include "module/identifier/models/shenzhen_0526.hpp"
#include "module/identifier/models/shenzhen_0708.hpp"
#include "module/identifier/models/tongji_yolov5.hpp"
#include "utility/image/image.details.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <print>
#include <ranges>

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

using namespace rmcs;
auto error_head = "[MODEL INFER ERROR]:\n";
auto location   = std::filesystem::path { __FILE__ }.parent_path();

constexpr auto config = R"(
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

// --- 资源路径 ---
// 测试资源路径配置:
// - 优先使用环境变量 TEST_ASSETS_ROOT
// - 未设置时默认使用 /tmp/auto_aim
// - 运行前需执行: cd test && ./download_assets.sh
AssetsManager assets_manager;

struct ExpectedCorners {
    float lt_x;
    float lt_y;
    float lb_x;
    float lb_y;
    float rb_x;
    float rb_y;
    float rt_x;
    float rt_y;

    auto lt() const noexcept -> cv::Point2f { return { lt_x, lt_y }; }
    auto lb() const noexcept -> cv::Point2f { return { lb_x, lb_y }; }
    auto rb() const noexcept -> cv::Point2f { return { rb_x, rb_y }; }
    auto rt() const noexcept -> cv::Point2f { return { rt_x, rt_y }; }
};

template <class model_type>
auto assert_sync_infer_with_expected(const Image& image,
    const std::array<ExpectedCorners, 2>& expected, bool use_roi_segment = false) -> void {
    const auto model_name     = std::string { model_type::kLocation };
    auto yaml                 = YAML::Load(config);
    const auto model_location = location / "../models" / model_name;
    yaml["model_location"]    = model_location.string();
    yaml["use_roi_segment"]   = use_roi_segment;

    auto detector         = identifier::ArmorDetection {};
    auto configure_result = detector.initialize(yaml);
    ASSERT_TRUE(configure_result.has_value())
        << error_head << model_name << " | " << configure_result.error();

    auto infer_begin   = std::chrono::steady_clock::now();
    auto detect_result = detector.sync_detect(image);
    auto infer_elapsed =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - infer_begin);
    ASSERT_TRUE(detect_result.has_value()) << error_head << model_name << " | detect failed";

    const auto& armors = detect_result.value();

    std::println();
    std::println("[ LOG      ] >>>>>>>> model: {}", model_name);
    std::println("[ LOG      ] infer cost: {:.3f} ms", infer_elapsed.count());

    ASSERT_EQ(armors.size(), expected.size())
        << error_head << model_name << " | armor count mismatch";

    auto matched   = std::array<bool, expected.size()> { false, false };
    auto threshold = 8.0;
    for (const auto&& [i, armor] : armors | std::views::enumerate) {
        auto lt = armor.tl;
        auto rt = armor.tr;
        auto rb = armor.br;
        auto lb = armor.bl;

        std::println("[ LOG      ] confidence {:2}: {:.3f}", i, armor.confidence);
        std::println("[ LOG      ]   lt=({:.1f}, {:.1f})  rt=({:.1f}, {:.1f})", //
            lt.x, lt.y, rt.x, rt.y);
        std::println("[ LOG      ]   rb=({:.1f}, {:.1f})  lb=({:.1f}, {:.1f})", //
            rb.x, rb.y, lb.x, lb.y);
        std::println("[ LOG      ]   color: {} genre: {}", get_enum_name(armor.color),
            get_enum_name(armor.genre));

        constexpr auto is_close = [](const auto& p, const auto& q, double tol) {
            return std::abs(p.x - q.x) <= tol && std::abs(p.y - q.y) <= tol;
        };

        for (std::size_t index = 0; index < expected.size(); index++) {
            if (matched.at(index)) continue;

            auto ok_lt = is_close(lt, expected.at(index).lt(), threshold);
            auto ok_lb = is_close(lb, expected.at(index).lb(), threshold);
            auto ok_rt = is_close(rt, expected.at(index).rt(), threshold);
            auto ok_rb = is_close(rb, expected.at(index).rb(), threshold);
            if (ok_lt && ok_lb && ok_rt && ok_rb) {
                matched[index] = true;
            }
        }
    }

    for (auto [i, result] : matched | std::views::enumerate) {
        EXPECT_TRUE(result) << error_head << model_name << " | "
                            << std::format("Armor {} not matched", i + 1);
    }
}

TEST(model, sync_infer) {
    const auto image_location = assets_manager.path("model_infer_example.jpg");
    auto image { Image {} };
    image.details().mat = cv::imread(image_location);
    ASSERT_FALSE(image.details().mat.empty())
        << error_head << std::format("Failed to read image from '{}'", image_location.string());

    constexpr auto expected = std::array {
        ExpectedCorners { 970.7f, 569.4f, 977.6f, 614.0f, 1057.8f, 615.0f, 1051.0f, 571.5f },
        ExpectedCorners { 697.7f, 580.1f, 690.2f, 619.0f, 751.4f, 620.9f, 758.9f, 581.4f },
    };

    assert_sync_infer_with_expected<TongJiYoloV5>(image, expected);
    assert_sync_infer_with_expected<ShenZhen0526>(image, expected);
    assert_sync_infer_with_expected<ShenZhen0708>(image, expected);
}

TEST(model, sync_infer_with_roi_segment) {
    const auto image_location = assets_manager.path("model_infer_example.jpg");
    auto image { Image {} };
    image.details().mat = cv::imread(image_location);
    ASSERT_FALSE(image.details().mat.empty())
        << error_head << std::format("Failed to read image from '{}'", image_location.string());

    constexpr auto expected = std::array {
        ExpectedCorners { 970.7f, 569.4f, 977.6f, 614.0f, 1057.8f, 615.0f, 1051.0f, 571.5f },
        ExpectedCorners { 697.7f, 580.1f, 690.2f, 619.0f, 751.4f, 620.9f, 758.9f, 581.4f },
    };

    assert_sync_infer_with_expected<TongJiYoloV5>(image, expected, true);
    assert_sync_infer_with_expected<ShenZhen0526>(image, expected, true);
    assert_sync_infer_with_expected<ShenZhen0708>(image, expected, true);
}

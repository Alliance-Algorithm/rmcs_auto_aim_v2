#include "module/identifier/model.hpp"
#include "utility/image/image.details.hpp"

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <print>
#include <ranges>

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

TEST(model, sync_infer) {
    using namespace rmcs::identifier;

    auto net  = OpenVinoNet {};
    auto yaml = YAML::Load(config);

    auto model_location    = location / "../models/yolov5.xml";
    yaml["model_location"] = model_location.string();

    auto result = net.configure(yaml);
    ASSERT_TRUE(result.has_value()) << error_head << result.error();

    const auto image_location = asset_path("model_infer_example.jpg");

    auto image { Image {} };
    image.details().mat = cv::imread(image_location);
    ASSERT_FALSE(image.details().mat.empty())
        << error_head << "Failed to read image from " + std::string(image_location);

    const auto use_roi_segment = yaml["use_roi_segment"].as<bool>();
    const auto roi_cols        = yaml["roi_cols"].as<int>();
    const auto roi_rows        = yaml["roi_rows"].as<int>();
    auto roi_offset            = cv::Point2f { 0.F, 0.F };
    if (use_roi_segment) {
        const auto width  = image.details().mat.cols;
        const auto height = image.details().mat.rows;
        if (width >= roi_cols && height >= roi_rows) {
            roi_offset = cv::Point2f {
                static_cast<float>((width - roi_cols) / 2.),
                static_cast<float>((height - roi_rows) / 2.),
            };
        }
    }

    auto infer_begin  = std::chrono::steady_clock::now();
    auto infer_result = net.sync_infer(image);
    auto infer_elapsed =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - infer_begin);
    std::println("[ LOG      ] infer cost: {:.3f} ms", infer_elapsed.count());

    ASSERT_TRUE(infer_result.has_value()) << error_head << infer_result.error();

    const auto& armors = infer_result.value();
    ASSERT_EQ(armors.size(), 2) << error_head << "The count of armor needs to be 2";

    constexpr auto expected = std::array {
        ArmorInferResult<>::Corners {
            970.7f, 569.4f, 977.6f, 614.0f, 1057.8f, 615.0f, 1051.0f, 571.5f },
        ArmorInferResult<>::Corners {
            697.7f, 580.1f, 690.2f, 619.0f, 751.4f, 620.9f, 758.9f, 581.4f },
    };

    auto matched   = std::array<bool, expected.size()> { false, false };
    auto threshold = use_roi_segment ? 5.0 : 2.0;
    for (const auto&& [i, armor] : infer_result.value() | std::views::enumerate) {
        auto lt = armor.corners.lt() + roi_offset;
        auto rt = armor.corners.rt() + roi_offset;
        auto rb = armor.corners.rb() + roi_offset;
        auto lb = armor.corners.lb() + roi_offset;

        std::println("[ LOG      ] confidence {:2}: {:.3f}", i, armor.confidence);
        std::println("[ LOG      ]   lt=({:.1f}, {:.1f})  rt=({:.1f}, {:.1f})", //
            lt.x, lt.y, rt.x, rt.y);
        std::println("[ LOG      ]   rb=({:.1f}, {:.1f})  lb=({:.1f}, {:.1f})", //
            rb.x, rb.y, lb.x, lb.y);
        std::println("[ LOG      ]   color: {} genre: {}", get_enum_name(armor.armor_color()),
            get_enum_name(armor.armor_genre()));

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
        EXPECT_TRUE(result) << error_head << std::format("Armor {} not matched", i + 1);
    }
}

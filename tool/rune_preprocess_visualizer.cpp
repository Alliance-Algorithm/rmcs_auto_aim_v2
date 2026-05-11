#include "module/identifier/rune/rune_preprocessor.hpp"
#include "utility/image/image.details.hpp"

#include <cstdlib>
#include <filesystem>
#include <format>
#include <print>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace {

struct DrawConfig {
    std::string image_filename = "red_buff_hit_active.png";
    bool show_window           = true;
    bool draw_index            = true;
    bool draw_area             = true;
};

auto discover_assets_root() -> std::filesystem::path {
    if (const char* root = std::getenv("TEST_ASSETS_ROOT"); root && *root) {
        return std::filesystem::path { root };
    }
    return "/tmp/auto_aim";
}

auto make_preprocessor_config() -> YAML::Node {
    auto config                = YAML::Node {};
    config["target_color"]     = "blue";
    config["binary_threshold"] = 40;
    config["min_contour_area"] = 20.0;
    config["max_contour_area"] = 10000.0;
    return config;
}

// 在原图上叠加预处理结果：
// 绿色矩形表示当前保留下来的候选轮廓外接框, 黄色圆点表示该候选的几何中心
auto draw_candidates(cv::Mat& canvas, const rmcs::identifier::RunePreprocessResult& result,
    const DrawConfig& config) -> void {
    for (std::size_t index = 0; index < result.candidates.size(); ++index) {
        const auto& candidate = result.candidates[index];

        // 候选外接框：用于直接观察当前参数最终保留下来的区域
        cv::rectangle(canvas, candidate.bounding_rect, cv::Scalar(0, 255, 0), 2);

        // 候选中心点：用于观察保留区域的中心位置是否稳定
        cv::circle(canvas, candidate.center, 3, cv::Scalar(0, 255, 255), cv::FILLED);

        // #编号 表示候选序号, area=... 表示该候选的轮廓面积
        auto label = std::string {};
        if (config.draw_index) label += std::format("#{}", index);

        if (config.draw_area) {
            if (!label.empty()) label += " ";
            label += std::format("area={:.1f}", candidate.area);
        }

        if (!label.empty()) {
            const auto text_origin = cv::Point { candidate.bounding_rect.x,
                std::max(15, candidate.bounding_rect.y - 8) };
            cv::putText(canvas, label, text_origin, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }
    }
}

} // namespace

auto main() -> int {
    auto draw_config         = DrawConfig {};
    auto preprocessor_config = make_preprocessor_config();

    const auto assets_root = discover_assets_root();
    const auto image_path  = assets_root / draw_config.image_filename;
    std::println("[rune_preprocess_visualizer] assets_root = {}", assets_root.string());
    std::println("[rune_preprocess_visualizer] image_filename = {}", draw_config.image_filename);
    std::println("[rune_preprocess_visualizer] image_path = {}", image_path.string());
    std::println("[rune_preprocess_visualizer] show_window = {}", draw_config.show_window);
    std::println("[rune_preprocess_visualizer] target_color = {}",
        preprocessor_config["target_color"].as<std::string>());
    std::println("[rune_preprocess_visualizer] binary_threshold = {}",
        preprocessor_config["binary_threshold"].as<int>());
    std::println("[rune_preprocess_visualizer] min_contour_area = {}",
        preprocessor_config["min_contour_area"].as<double>());
    std::println("[rune_preprocess_visualizer] max_contour_area = {}",
        preprocessor_config["max_contour_area"].as<double>());

    if (!std::filesystem::exists(image_path)) {
        std::println("[rune_preprocess_visualizer] image does not exist: {}", image_path.string());
        return EXIT_FAILURE;
    }

    auto image          = rmcs::Image {};
    image.details().mat = cv::imread(image_path.string());
    if (image.details().mat.empty()) {
        std::println("[rune_preprocess_visualizer] failed to read image: {}", image_path.string());
        return EXIT_FAILURE;
    }

    auto preprocessor = rmcs::identifier::RunePreprocessor {};
    if (auto result = preprocessor.initialize(preprocessor_config); !result.has_value()) {
        std::println("[rune_preprocess_visualizer] initialize failed: {}", result.error());
        return EXIT_FAILURE;
    }

    auto preprocess_result = preprocessor.sync_process(image);
    if (!preprocess_result.has_value()) {
        std::println("[rune_preprocess_visualizer] preprocess failed");
        return EXIT_FAILURE;
    }

    std::println(
        "[rune_preprocess_visualizer] candidates = {}", preprocess_result->candidates.size());

    // 窗口展示“原图 + 候选框/中心点/标签”叠加效果
    auto painted = image.details().mat.clone();
    draw_candidates(painted, *preprocess_result, draw_config);

    if (draw_config.show_window) {
        cv::imshow("rune_preprocess_visualizer", painted);
        std::println("[rune_preprocess_visualizer] press any key in the image window to exit");
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    return EXIT_SUCCESS;
}

#include "module/identifier/rune/rune_feature_identifier.hpp"
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
    enum class ViewMode {
        PREPROCESS,
        CENTER,
    };
    // 替换为测试图片的地址
    std::string image_filename = "blue_buff_active.png";
    bool show_window           = true;
    ViewMode view_mode         = ViewMode::CENTER;
};

auto discover_assets_root() -> std::filesystem::path {
    if (const char* root = std::getenv("TEST_ASSETS_ROOT"); root && *root) {
        return std::filesystem::path { root };
    }
    return "/tmp/auto_aim";
}

auto load_identifier_config() -> YAML::Node {
    const auto config_path =
        std::filesystem::path { __FILE__ }.parent_path().parent_path() / "config" / "config.yaml";
    return YAML::LoadFile(config_path.string())["identifier"];
}

auto load_image(const std::filesystem::path& image_path) -> std::unique_ptr<rmcs::Image> {
    if (!std::filesystem::exists(image_path)) {
        return nullptr;
    }

    auto image           = std::make_unique<rmcs::Image>();
    image->details().mat = cv::imread(image_path.string());
    if (image->details().mat.empty()) {
        return nullptr;
    }

    return image;
}

// 在原图上叠加预处理结果：
// 绿色矩形表示当前保留下来的候选轮廓外接框, 黄色圆点表示该候选的几何中心
auto draw_candidates(cv::Mat& canvas, const rmcs::identifier::RunePreprocessResult& result)
    -> void {
    for (std::size_t index = 0; index < result.candidates.size(); ++index) {
        const auto& candidate = result.candidates[index];

        // 候选外接框：用于直接观察当前参数最终保留下来的区域
        cv::rectangle(canvas, candidate.bounding_rect, cv::Scalar(0, 255, 0), 2);

        // 候选中心点：用于观察保留区域的中心位置是否稳定
        cv::circle(canvas, candidate.center, 3, cv::Scalar(0, 255, 255), cv::FILLED);

        // #编号 表示候选序号, area=... 表示该候选的轮廓面积
        const auto label = std::format("#{} area={:.1f}", index, candidate.area);
        const auto text_origin =
            cv::Point { candidate.bounding_rect.x, std::max(15, candidate.bounding_rect.y - 8) };
        cv::putText(canvas, label, text_origin, cv::FONT_HERSHEY_SIMPLEX, 0.5,
            cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
}

auto draw_centers(cv::Mat& canvas, const rmcs::identifier::RuneFeatureIdentifyResult& result)
    -> void {
    for (const auto& center : result.centers) {
        cv::Point2f corners[4];
        center.rotated_rect.points(corners);
        for (int index = 0; index < 4; ++index) {
            cv::line(canvas, corners[index], corners[(index + 1) % 4], cv::Scalar(255, 0, 255), 2,
                cv::LINE_AA);
        }

        cv::circle(canvas, center.center, 4, cv::Scalar(0, 255, 255), cv::FILLED);
    }
}

} // namespace

auto main() -> int {
    auto draw_config         = DrawConfig {};
    auto identifier_config   = load_identifier_config();
    auto preprocessor_config = identifier_config["rune_preprocessor"];
    auto feature_config      = identifier_config["rune_feature_identifier"];

    const auto assets_root = discover_assets_root();
    const auto image_path  = assets_root / draw_config.image_filename;

    auto image = load_image(image_path);
    if (!image) {
        std::println("[rune_preprocess_visualizer] failed to load image: {}", image_path.string());
        return EXIT_FAILURE;
    }

    auto preprocessor = rmcs::identifier::RunePreprocessor {};
    if (auto result = preprocessor.initialize(preprocessor_config); !result.has_value()) {
        std::println("[rune_preprocess_visualizer] initialize failed: {}", result.error());
        return EXIT_FAILURE;
    }

    auto feature_identifier = rmcs::identifier::RuneFeatureIdentifier {};
    if (auto result = feature_identifier.initialize(feature_config); !result.has_value()) {
        std::println("[rune_preprocess_visualizer] feature initialize failed: {}", result.error());
        return EXIT_FAILURE;
    }

    auto preprocess_result = preprocessor.sync_process(*image);
    if (!preprocess_result.has_value()) {
        std::println("[rune_preprocess_visualizer] preprocess failed");
        return EXIT_FAILURE;
    }

    std::println(
        "[rune_preprocess_visualizer] candidates = {}", preprocess_result->candidates.size());

    auto painted = image->details().mat.clone();
    if (draw_config.view_mode == DrawConfig::ViewMode::PREPROCESS) {
        draw_candidates(painted, *preprocess_result);
    } else {
        auto feature_result = feature_identifier.sync_identify(*preprocess_result);
        if (!feature_result.has_value()) {
            std::println(
                "[rune_preprocess_visualizer] center identification failed: {}",
                feature_result.error());
            return EXIT_FAILURE;
        }

        std::println("[rune_preprocess_visualizer] centers = {}", feature_result->centers.size());
        draw_centers(painted, *feature_result);
    }

    if (draw_config.show_window) {
        cv::imshow("rune_preprocess_visualizer", painted);
        std::println("[rune_preprocess_visualizer] press any key in the image window to exit");
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    return EXIT_SUCCESS;
}

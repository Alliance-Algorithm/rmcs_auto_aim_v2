#pragma once
#include "utility/pimpl.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

#include <opencv2/core/mat.hpp>

#include <expected>
#include <string_view>

namespace rmcs::identifier {

struct NeuralNetwork final {
    RMCS_PIMPL_DEFINITION(NeuralNetwork)

public:
    using ArmorId   = DeviceId;
    using ArmorRect = cv::Rect;

    using Input = cv::Mat;

    struct Result {
        ArmorId id = ArmorId::UNKNOWN;
        ArmorRect rect;
    };
    struct Config {
        CampColor color = CampColor::UNKNOWN;

        cv::Size2i input_size = { 1440, 1080 };
        cv::Size2i infer_size = { 640, 640 };

        double confidence_threshold          = 0.65;
        double non_max_suppression_threshold = 0.45;
    };

    auto configure(const Config&) noexcept -> std::expected<void, std::string>;

    auto load_from_filesystem(std::string_view location) noexcept
        -> std::expected<void, std::string_view>;

    auto make_inference(const Input& input) const noexcept -> std::vector<Result>;
};

}

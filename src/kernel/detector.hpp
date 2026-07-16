#pragma once

#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/rune.hpp"

#include <expected>
#include <optional>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <yaml-cpp/node/node.h>

namespace rmcs::kernel {

class Detector {
    RMCS_PIMPL_DEFINITION(Detector)

public:
    struct Result {
        Armor2ds armors;

        Lightbar2ds lightbars;
        std::optional<cv::Rect2i> green_light;

        std::vector<RuneIcon> icons;
        std::vector<RuneBullseye> bullseyes;

        std::vector<cv::Rect2i> areas;
    };

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto update_detect_color(CampColor) noexcept -> void;
    auto update_detect_rune(bool) noexcept -> void;
    auto update_camera(const std::array<double, 9>&) noexcept -> void;
    auto update_camera(const std::array<double, 5>&) noexcept -> void;

    auto detect(const cv::Mat&) noexcept -> Result;
};

}

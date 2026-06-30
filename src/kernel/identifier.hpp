#pragma once

#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

#include <expected>
#include <optional>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <yaml-cpp/node/node.h>

namespace rmcs::kernel {

class Identifier {
    RMCS_PIMPL_DEFINITION(Identifier)

public:
    struct Result {
        Armor2ds armors;

        Lightbar2ds lightbars;
        std::optional<cv::Rect2i> green_light;

        std::vector<cv::Rect2i> areas;
    };

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto sync_identify(const cv::Mat&) noexcept -> std::optional<Result>;
};

}

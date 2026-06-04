#pragma once

#include "utility/image/image.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

#include <expected>
#include <optional>

#include <opencv2/core/types.hpp>
#include <yaml-cpp/node/node.h>

namespace rmcs::kernel {

class Identifier {
    RMCS_PIMPL_DEFINITION(Identifier)

public:
    struct Result {
        Armor2Ds armors;
    };

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto sync_identify(const Image&) noexcept -> std::optional<Result>;
    auto draw_green_light(Image&) noexcept -> void;
    auto draw_green_light_roi(Image&) noexcept -> void;
};

}

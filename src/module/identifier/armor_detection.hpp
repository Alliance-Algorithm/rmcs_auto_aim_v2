#pragma once

#include "utility/image/image.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

#include <expected>

#include <opencv2/core/types.hpp>
#include <yaml-cpp/yaml.h>

namespace rmcs::identifier {

class ArmorDetection {
    RMCS_PIMPL_DEFINITION(ArmorDetection)

public:
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;
    auto sync_detect(const Image&) noexcept -> std::optional<std::vector<Armor2D>>;
};

}

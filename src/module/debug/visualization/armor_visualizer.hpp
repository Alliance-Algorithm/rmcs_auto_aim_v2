#pragma once

#include "utility/pimpl.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::debug {

class ArmorVisualizer {

    RMCS_PIMPL_DEFINITION(ArmorVisualizer)

public:
    auto initialize(util::RclcppNode&) noexcept -> void;

    auto visualize(
        std::span<Armor3D const> armors, std::string_view name, std::string_view link_name) -> bool;
};
}

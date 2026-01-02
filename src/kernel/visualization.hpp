#pragma once
#include "utility/image/image.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/robot/armor.hpp"

#include <expected>
#include <yaml-cpp/yaml.h>

namespace rmcs::kernel {

class Visualization {
    RMCS_PIMPL_DEFINITION(Visualization)

public:
    static constexpr auto get_prefix() noexcept { return "visualization"; }

    auto operator<<(const Image& image) noexcept -> Visualization& {
        return send_image(image), *this;
    }

public:
    auto initialize(const YAML::Node& yaml, util::RclcppNode& visual_node) noexcept
        -> std::expected<void, std::string>;

    auto initialized() const noexcept -> bool;

    auto send_image(const Image& image) noexcept -> bool;

    auto solved_pnp_armors(std::span<Armor3D const> armors) const -> bool;
    auto predicted_armors(std::span<Armor3D const> armors) const -> bool;
};

}
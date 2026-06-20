#pragma once

#include <expected>
#include <optional>
#include <span>
#include <string>

#include <yaml-cpp/yaml.h>

#include "module/fire_control/types.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::fire_control {

class ArmorSelector {
    RMCS_PIMPL_DEFINITION(ArmorSelector)

public:
    struct Config {
        double coming_angle;
        double leaving_angle;
        double outpost_coming_angle;
        double outpost_leaving_angle;
        double switch_threshold;
    };

    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string>;

    auto select(std::span<ArmorCandidate const> candidates,
        std::optional<int> last_selected_armor_id) const -> std::optional<size_t>;
};

} // namespace rmcs::fire_control

#pragma once

#include <chrono>
#include <expected>
#include <yaml-cpp/yaml.h>

#include "module/tracker/decider.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::kernel {

struct Tracker {
    RMCS_PIMPL_DEFINITION(Tracker)
public:
    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string>;

    auto filter_armors(std::span<Armor2D> const& armors) const -> std::vector<Armor2D>;

    auto decide(std::span<Armor3D> const& armors, std::chrono::steady_clock::time_point t)
        -> tracker::Decider::Output;
};
}

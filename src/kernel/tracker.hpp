#pragma once

#include <expected>
#include <span>
#include <yaml-cpp/yaml.h>

#include "module/tracker/decider.hpp"
#include "utility/clock.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::kernel {

struct Tracker {
    RMCS_PIMPL_DEFINITION(Tracker)

public:
    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string>;

    auto set_invincible_armors(DeviceIds devices) -> void;

    auto filter_armors(std::span<Armor2D> armors) const -> std::vector<Armor2D>;

    auto decide(std::span<Armor3D const> armors, TimePoint t) -> tracker::Decider::Output;
};
}

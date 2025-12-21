#pragma once

#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"
namespace rmcs::tracker {
class ArmorFilter {

    RMCS_PIMPL_DEFINITION(ArmorFilter)
public:
    auto set_enemy_color(CampColor const& color) -> void;

    auto set_invincible_armors(std::span<DeviceId> const&) -> void;

    auto filter(std::span<Armor2D> const&) const -> std::vector<Armor2D>;
};
}

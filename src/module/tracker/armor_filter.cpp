#include "armor_filter.hpp"
#include "utility/robot/armor.hpp"

using namespace rmcs::tracker;

struct ArmorFilter::Impl {
    auto set_enemy_color(CampColor const& color) -> void { enemy_color = color; }

    auto set_invincible_armors(DeviceIds devices) -> void { invincible_armors = devices; }

    auto filter(std::span<Armor2D> const& armors) const -> std::vector<Armor2D> {
        return armors | std::views::filter([&](Armor2D const& armor) {
            return (armor.genre != DeviceId::INFANTRY_5)
                && (armor_color2camp_color(armor.color) == enemy_color)
                && (!invincible_armors.contains(armor.genre));
        }) | std::ranges::to<std::vector>();
    }

    CampColor enemy_color { CampColor::UNKNOWN };
    DeviceIds invincible_armors { DeviceIds::None() };
};

ArmorFilter::ArmorFilter() noexcept
    : pimpl(std::make_unique<Impl>()) { }
ArmorFilter::~ArmorFilter() noexcept = default;

auto ArmorFilter::set_enemy_color(CampColor color) -> void { return pimpl->set_enemy_color(color); }

auto ArmorFilter::set_invincible_armors(DeviceIds devices) -> void {
    return pimpl->set_invincible_armors(devices);
}

auto ArmorFilter::filter(std::span<Armor2D> const& armors) const -> std::vector<Armor2D> {
    return pimpl->filter(armors);
}

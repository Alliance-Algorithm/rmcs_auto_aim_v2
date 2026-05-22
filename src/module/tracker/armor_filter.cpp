#include "armor_filter.hpp"
#include "utility/robot/armor.hpp"

using namespace rmcs::tracker;

struct ArmorFilter::Impl {
    auto set_enemy_color(CampColor const& color) -> void { enemy_color = color; }

    auto set_invincible_armors(DeviceIds devices) -> void { invincible_armors = devices; }

    auto set_target_mode(TargetMode mode) -> void { target_mode = mode; }

    auto filter(std::span<Armor2D> const& armors) const -> std::vector<Armor2D> {
        auto allowed_targets = allowed_target_ids(target_mode);
        auto filtered = armors | std::views::filter([&](Armor2D const& armor) {
            return (armor.genre != DeviceId::INFANTRY_5) && (armor.genre != DeviceId::UNKNOWN)
                && allowed_targets.contains(armor.genre)
                && (armor_color2camp_color(armor.color) == enemy_color)
                && (!invincible_armors.contains(armor.genre));
        });
        return std::ranges::to<std::vector>(filtered);
    }

    CampColor enemy_color { CampColor::UNKNOWN };
    DeviceIds invincible_armors { DeviceIds::None() };
    TargetMode target_mode { TargetMode::COMBAT };
};

ArmorFilter::ArmorFilter() noexcept
    : pimpl(std::make_unique<Impl>()) { }
ArmorFilter::~ArmorFilter() noexcept = default;

auto ArmorFilter::set_enemy_color(CampColor color) -> void { return pimpl->set_enemy_color(color); }

auto ArmorFilter::set_invincible_armors(DeviceIds devices) -> void {
    return pimpl->set_invincible_armors(devices);
}

auto ArmorFilter::set_target_mode(TargetMode mode) -> void { return pimpl->set_target_mode(mode); }

auto ArmorFilter::filter(std::span<Armor2D> const& armors) const -> std::vector<Armor2D> {
    return pimpl->filter(armors);
}

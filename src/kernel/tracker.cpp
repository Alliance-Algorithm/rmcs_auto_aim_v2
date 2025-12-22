#include "tracker.hpp"

#include "module/tracker/armor_filter.hpp"
#include "module/tracker/decider.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::kernel;
using namespace rmcs::tracker;

struct Tracker::Impl {
    ArmorFilter filter;
    Decider decider;

    struct Config : util::Serializable {
        std::string enemy_color;
        constexpr static std::tuple metas { &Config::enemy_color, "enemy_color" };
    };

    Config config;

    bool found { false };
    DeviceIds device_candidates { DeviceIds::None() };

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        if (config.enemy_color == "red") {
            filter.set_enemy_color(CampColor::RED);
        } else if (config.enemy_color == "blue") {
            filter.set_enemy_color(CampColor::BLUE);
        } else {
            return std::unexpected { "enemy_color 应该是 [blue] or [red]." };
        }

        return {};
    }

    auto set_invincible_armors(DeviceIds devices) -> void {
        return filter.set_invincible_armors(devices);
    }

    auto filter_armors(std::span<Armor2D> const& armors) const -> std::vector<Armor2D> {
        auto result = filter.filter(armors);
        return result;
    }

    auto decide(std::span<Armor3D const> armors, Clock::time_point t) -> Decider::Output {
        auto decider_output = decider.update(armors, t);
        return decider_output;
    }

    // TODO:need to choose armor by priority
};

Tracker::Tracker() noexcept
    : pimpl(std::make_unique<Impl>()) { }
Tracker::~Tracker() noexcept = default;

auto Tracker::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto Tracker::set_invincible_armors(DeviceIds devices) -> void {
    return pimpl->set_invincible_armors(devices);
}

auto Tracker::filter_armors(std::span<Armor2D> armors) const -> std::vector<Armor2D> {
    return pimpl->filter_armors(armors);
}

auto Tracker::decide(std::span<Armor3D const> armors, Clock::time_point t) -> Decider::Output {
    return pimpl->decide(armors, t);
}

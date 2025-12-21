#include "tracker.hpp"

#include "module/tracker/armor_filter.hpp"
#include "utility/serializable.hpp"
using namespace rmcs::kernel;
using namespace rmcs::tracker;

struct Tracker::Impl {
    ArmorFilter filter;

    struct Config : util::Serializable {
        std::string enemy_color;
        constexpr static std::tuple metas {
            &Config::enemy_color,
            "enemy_color",
        };
    };

    Config config;

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
            return std::unexpected { "enemy_color应该是[blue] or [red]." };
        }

        return {};
    }

    auto filter_armors(std::span<Armor2D> const& armors) const -> auto {
        return filter.filter(armors);
    }
};

Tracker::Tracker() noexcept
    : pimpl(std::make_unique<Impl>()) { }
Tracker::~Tracker() noexcept = default;

auto Tracker::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto Tracker::filter_armors(std::span<Armor2D> const& armors) const -> std::vector<Armor2D> {
    return pimpl->filter_armors(armors);
}

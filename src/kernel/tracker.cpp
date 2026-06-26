#include "tracker.hpp"

#include "module/tracker/armor_filter.hpp"
#include "module/tracker/decider.hpp"
#include "utility/serializable.hpp"

#include <unordered_map>

using std::move;

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

    static auto remove_duplicate_armors(const std::vector<Armor2d>& armors) {

        auto best_armors = std::unordered_map<ArmorGenre, Armor2d> { };

        for (const auto& armor : armors) {
            auto left_center  = 0.5f * (armor.tl + armor.bl);
            auto right_center = 0.5f * (armor.tr + armor.br);
            auto distance     = cv::norm(right_center - left_center);

            auto it = best_armors.find(armor.genre);
            if (it == best_armors.end()) {
                best_armors[armor.genre] = armor;
            } else {
                auto existing_left  = 0.5f * (it->second.tl + it->second.bl);
                auto existing_right = 0.5f * (it->second.tr + it->second.br);
                auto existing_dist  = cv::norm(existing_right - existing_left);
                if (distance > existing_dist) {
                    it->second = armor;
                }
            }
        }

        auto result = std::vector<Armor2d> { };
        result.reserve(best_armors.size());
        for (auto& [genre, armor] : best_armors) {
            result.push_back(armor);
        }
        return result;
    }

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

        result = decider.initialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        return { };
    }

    auto set_invincible_armors(DeviceIds devices) { filter.set_invincible_armors(devices); }

    auto set_enemy_color(CampColor color) { filter.set_enemy_color(color); }

    auto filter_armors(std::span<Armor2d> const& armors) const -> std::vector<Armor2d> {
        auto result = filter.filter(armors);
        return result;
    }

    auto decide(std::span<Armor3d const> armors, TimePoint t) -> Decider::Output {
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
    pimpl->set_invincible_armors(devices);
}

auto Tracker::set_enemy_color(CampColor color) -> void { pimpl->set_enemy_color(color); }

auto Tracker::filter_armors(std::span<Armor2d> armors) const -> std::vector<Armor2d> {
    return pimpl->filter_armors(armors);
}

auto Tracker::decide(std::span<Armor3d const> armors, TimePoint t) -> Decider::Output {
    return pimpl->decide(armors, t);
}

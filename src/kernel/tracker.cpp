#include "tracker.hpp"

#include "module/predictor/model/outpost.hpp"
#include "module/predictor/model/robot.hpp"
#include "module/tracker/armor_filter.hpp"
#include "module/tracker/decider.hpp"
#include "utility/logging/printer.hpp"
#include "utility/serializable.hpp"

#include <unordered_map>

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

struct TrackerV2::Impl {
    struct Config : util::Serializable {
        std::string fallback_color = "RED";
        double timeout_seconds     = 1.5;

        static constexpr std::tuple metas {
            // clang-format off
            &Config::fallback_color, "fallback_color",
            // clang-format on
        };
    } config;

    ArmorColor track_color = ArmorColor::DARK;

    struct Stored {
        Armor2ds armor2ds;
        Armor3ds armor3ds;
        Lightbar2ds lightbar2ds;
    } stored;

    Timestamp stored_stamp;

    RobotModel::Config robot_config;
    std::unordered_map<ArmorGenre, Timestamp> robot_stamps;
    std::unordered_map<ArmorGenre, RobotModel> robots;

    OutpostModel::Config outpost_config;
    Timestamp outpost_stamp;
    std::unique_ptr<OutpostModel> outpost;

    /* 假装这里有一个大符的 Model */

    Printer logging { "TrackerV2" };

    explicit Impl(const YAML::Node& yaml) {
        if (auto ret = config.serialize(yaml); !ret) {
            logging.error("TrackerV2 初始化错误: {}", ret.error());
            throw std::runtime_error { "无法构造 TrackerV2" };
        }
    }

    auto clean(Timestamp stamp) noexcept {
        stored.armor2ds.clear();
        stored.armor3ds.clear();
        stored.lightbar2ds.clear();

        stored_stamp = stamp;
    }

    auto store(std::span<const Armor2d> items) {
        for (const auto& item : items) {
            if (item.color == track_color) {
                stored.armor2ds.push_back(item);
            }
        }
    }
    auto store(std::span<const Armor3d> items) {
        for (const auto& item : items) {
            if (item.color == track_color) {
                stored.armor3ds.push_back(item);
            }
        }
    }
    auto store(std::span<const Lightbar2d> items) {
        for (const auto& item : items) {
            if (item.color == track_color) {
                stored.lightbar2ds.push_back(item);
            }
        }
    }

    auto execute() {
        { // 前哨站观测超时
            const auto dt = stored_stamp - outpost_stamp;
            const auto s  = std::chrono::duration<double> { dt };
            if (s.count() > config.timeout_seconds) {
                outpost = nullptr;
            }
        }
        // 机器人观测超时处理
        for (const auto [id, stamp] : robot_stamps) {
            const auto dt = stored_stamp - stamp;
            const auto s  = std::chrono::duration<double> { dt };
            if (s.count() > config.timeout_seconds) {
                robots.erase(id);
            }
        }

        struct Target final {
            std::vector<Armor2d> armor2ds;
            std::vector<Armor3d> armor3ds;
            std::vector<Lightbar2d> bars;
        };
        auto seen = std::unordered_map<ArmorGenre, Target> { };

        for (const auto& armor : stored.armor2ds) {
            seen[armor.genre].armor2ds.push_back(armor);
        }
        for (const auto& armor : stored.armor3ds) {
            seen[armor.genre].armor3ds.push_back(armor);
        }
        for (const auto& bar : stored.lightbar2ds) {
            seen[bar.genre].bars.push_back(bar);
        }

        {
            constexpr auto kId = ArmorGenre::OUTPOST;
            if (seen.contains(kId)) {
                const auto& armors = seen[kId].armor3ds;
                if (!armors.empty()) {
                    if (outpost == nullptr) {
                        outpost = std::make_unique<OutpostModel>(armors.front());
                        outpost->configure(outpost_config);

                        robot_stamps[kId] = stored_stamp;
                    } else {
                        const auto dt = stored_stamp - robot_stamps[kId];
                        const auto s  = std::chrono::duration<double> { dt };
                    }
                }
                seen.erase(kId);
            }
        }

        for (const auto& [id, target] : seen) {
            if (robots.contains(id)) {
                robots.try_emplace(id, robot_config);
                robots[id].start_with(target.armor2ds);
            } else {
            }
        }

        return nullptr;
    }
};

TrackerV2::TrackerV2(const YAML::Node& yaml)
    : pimpl { std::make_unique<Impl>(yaml) } { }

TrackerV2::~TrackerV2() noexcept = default;

auto TrackerV2::clean(Timestamp stamp) noexcept -> void { pimpl->clean(stamp); }

auto TrackerV2::store(std::span<const Armor2d> item) -> void { pimpl->store(item); }
auto TrackerV2::store(std::span<const Armor3d> item) -> void { pimpl->store(item); }
auto TrackerV2::store(std::span<const Lightbar2d> item) -> void { pimpl->store(item); }

auto TrackerV2::execute() -> Trackable::Unique { return pimpl->execute(); }

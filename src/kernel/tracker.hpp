#pragma once

#include <expected>
#include <span>
#include <yaml-cpp/yaml.h>

#include "module/predictor/trackable.hpp"
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

    auto set_enemy_color(CampColor color) -> void;

    auto filter_armors(std::span<Armor2d> armors) const -> std::vector<Armor2d>;

    auto decide(std::span<Armor3d const> armors, TimePoint t) -> tracker::Decider::Output;
};

class TrackerV2 {
    RMCS_PIMPL_DEFINITION(TrackerV2)

public:
    struct Addition {
        Armor2ds tracked2d;
        Armor3ds tracked3d;
    };

    explicit TrackerV2(const YAML::Node&);

    auto update_track_color(CampColor) -> void;

    auto update_camera(const Transform&) noexcept -> void;
    auto update_camera(const std::array<double, 9>&) noexcept -> void;
    auto update_camera(const std::array<double, 5>&) noexcept -> void;

    auto clean() noexcept -> void;

    auto store(std::span<const Armor2d>) -> void;
    auto store(std::span<const Armor3d>) -> void;
    auto store(std::span<const Lightbar2d>) -> void;

    auto execute(Timestamp) -> Trackable::Unique;

    auto addition() const -> const Addition&;
};

}

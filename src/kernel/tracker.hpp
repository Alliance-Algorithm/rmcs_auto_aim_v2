#pragma once

#include <optional>
#include <span>
#include <yaml-cpp/yaml.h>

#include "module/predictor/trackable.hpp"
#include "utility/clock.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::kernel {

class TrackerV2 {
    RMCS_PIMPL_DEFINITION(TrackerV2)

public:
    struct Addition {
        Armor2ds tracked2d;
        Armor3ds tracked3d;

        struct Lightbar {
            int id;
            Point2d point;
        };
        std::vector<Lightbar> lightbars;
    };

    explicit TrackerV2(const YAML::Node&);

    auto update_track_color(CampColor) -> void;
    auto update_track_genre(DeviceIds) -> void;

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

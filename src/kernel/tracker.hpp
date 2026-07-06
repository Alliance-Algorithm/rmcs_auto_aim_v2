#pragma once

#include "module/tracker/trackable.hpp"
#include "utility/clock.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

#include <span>
#include <yaml-cpp/yaml.h>

namespace rmcs::kernel {

class Tracker {
    RMCS_PIMPL_DEFINITION(Tracker)

public:
    struct Addition {
        Armor2ds tracked2d;
        Armor3ds tracked3d;

        struct Lightbar {
            int id;
            Point2d point;
        };
        std::vector<Lightbar> lightbars;

        struct Info {
            std::string text;
            Point3d point;
        };
        std::vector<Info> infos;
    };

    explicit Tracker(const YAML::Node&);

    auto update_aim_intent(bool intent) -> void;

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

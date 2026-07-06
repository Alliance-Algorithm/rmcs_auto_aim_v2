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

    /// @brief:
    ///  设置自瞄意图，当关闭自瞄时，选取距离相机 X 轴最近的
    ///  机器人锁定，当自瞄开启时，锁定该机器人，即使该机器
    ///  人离开视野
    auto update_aim_intent(bool intent) -> void;

    /// @brief:
    ///  设置锁定超时，开启时，锁定对象将会被模型超时所清理，
    ///  此时会按照正常的优先级选取下一个目标，关闭时，一旦
    ///  锁定，则永远只瞄准该目标，直到自瞄意图被关闭
    auto update_aim_cleanup(bool on) -> void;

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

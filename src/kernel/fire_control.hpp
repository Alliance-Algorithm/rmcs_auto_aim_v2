#pragma once
#include "module/tracker/trackable.hpp"
#include "utility/pimpl.hpp"
#include "utility/serializable.hpp"

#include <optional>

#include <yaml-cpp/yaml.h>

namespace rmcs::kernel {

class FireController {
    RMCS_PIMPL_DEFINITION(FireController)

public:
    struct Config : util::Serializable {
        double bullet_speed;
        double shoot_delay;
        double offset_yaw { 0.0 };
        double offset_pitch { 0.0 };

        double attack_window { 40.0 };

        double degraded_angle_speed { 12.0 };

        double window_hysteresis { 0.2 };

        double yaw_tolerance   = 0.07;
        double pitch_tolerance = 0.04;

        bool require_stable_command = true;
        bool is_lazy_gimbal         = false;
        bool attack_preaim          = false;

        static constexpr std::tuple metas {
            // clang-format off
            &Config::bullet_speed, "bullet_speed",
            &Config::shoot_delay, "shoot_delay",
            &Config::offset_yaw, "offset_yaw",
            &Config::offset_pitch, "offset_pitch",
            &Config::attack_window, "attack_window",
            &Config::degraded_angle_speed, "degraded_angle_speed",
            &Config::window_hysteresis, "window_hysteresis",
            &Config::yaw_tolerance, "yaw_tolerance",
            &Config::pitch_tolerance, "pitch_tolerance",
            &Config::require_stable_command, "require_stable_command",
            &Config::is_lazy_gimbal, "is_lazy_gimbal",
            &Config::attack_preaim, "attack_preaim",
            // clang-format on
        };
    };

    struct State {
        Timestamp timestamp = { };

        double yaw   = 0.;
        double pitch = 0.;

        double max_yaw_vel = 10.0;
        double max_yaw_acc = 200.0;
    };

    struct Aimed {
        double aim_yaw = 0;
        double pitch   = 0;

        bool shoot   = false;
        bool pre_aim = false;

        Direction3d target;
        Point3d center;
        Point3d attack;
    };

    explicit FireController(const Config&);

    auto update(State state) -> void;

    auto aim(const Trackable&) -> std::optional<Aimed>;
};

}

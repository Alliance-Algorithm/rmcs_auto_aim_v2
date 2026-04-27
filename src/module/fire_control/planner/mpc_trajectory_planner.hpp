#pragma once

#include <expected>
#include <optional>

#include "module/predictor/snapshot.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::fire_control {

class MpcTrajectoryPlanner {
    RMCS_PIMPL_DEFINITION(MpcTrajectoryPlanner)

public:
    struct Config {
        bool mpc_enable { true };
        double mpc_max_yaw_acc { 50.0 };
        double mpc_yaw_q_angle { 9e6 };
        double mpc_yaw_q_rate { 0.0 };
        double mpc_yaw_r_acc { 1.0 };
        double mpc_max_pitch_acc { 100.0 };
        double mpc_pitch_q_angle { 9e6 };
        double mpc_pitch_q_rate { 0.0 };
        double mpc_pitch_r_acc { 1.0 };

        double coming_angle { 0.0 };
        double leaving_angle { 0.0 };
        double outpost_coming_angle { 0.0 };
        double outpost_leaving_angle { 0.0 };
    };

    struct Plan {
        double target_yaw { 0.0 };
        double target_pitch { 0.0 };
        double yaw { 0.0 };
        double pitch { 0.0 };
    };

    auto initialize(Config const& config) noexcept -> std::expected<void, std::string>;

    auto plan(const predictor::Snapshot& snapshot, TimePoint center_time, double bullet_speed,
        double yaw_offset, double pitch_offset) -> std::optional<Plan>;
};

} // namespace rmcs::fire_control

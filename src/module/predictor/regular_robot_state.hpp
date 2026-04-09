#pragma once

#include <chrono>
#include <span>
#include <vector>

#include "module/predictor/ekf_parameter.hpp"
#include "module/predictor/snapshot.hpp"

namespace rmcs::predictor {

class RegularRobotState {
public:
    using Clock = util::Clock;
    using EKF   = EKFParameters::EKF;

    struct MatchResult {
        int armor_id;
        double error;
        bool is_valid;
    };

    explicit RegularRobotState(Clock::time_point stamp = Clock::now()) noexcept;

    auto initialize(Armor3D const& armor, Clock::time_point t) -> void;
    auto predict(Clock::time_point t) -> void;

    auto match(Armor3D const& armor) const -> MatchResult;
    auto update(Armor3D const& armor) -> bool;
    auto update(std::span<Armor3D const> armors) -> bool;

    auto is_converged() const -> bool;
    auto get_snapshot() const -> Snapshot;
    auto distance() const -> double;

private:
    auto calculate_armors(EKF::XVec const& x) const -> std::vector<Eigen::Vector4d>;

    DeviceId device { DeviceId::UNKNOWN };
    CampColor color { CampColor::UNKNOWN };
    int armor_num { 0 };

    EKF ekf { EKF {} };
    Clock::time_point time_stamp;

    bool initialized { false };
    int update_count { 0 };

    const std::chrono::duration<double> reset_interval { 1.0 };
    const double angle_error_threshold { 0.65 };
};

} // namespace rmcs::predictor

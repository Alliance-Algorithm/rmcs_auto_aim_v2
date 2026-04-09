#pragma once

#include <chrono>
#include <span>

#include "module/predictor/outpost_state.hpp"
#include "module/predictor/snapshot.hpp"

namespace rmcs::predictor {

class OutpostRobotState {
public:
    using Clock = util::Clock;
    using EKF   = OutpostEKFParameters::EKF;

    struct MatchResult {
        int armor_id;
        double error;
        bool is_valid;
    };

    explicit OutpostRobotState(Clock::time_point stamp = Clock::now()) noexcept;

    auto initialize(Armor3D const& armor, Clock::time_point t) -> void;
    auto predict(Clock::time_point t) -> void;

    auto match(Armor3D const& armor) const -> MatchResult;
    auto update(Armor3D const& armor) -> bool;
    auto update(std::span<Armor3D const> armors) -> bool;

    auto is_converged() const -> bool;
    auto get_snapshot() const -> Snapshot;
    auto distance() const -> double;

private:
    CampColor color { CampColor::UNKNOWN };
    int armor_num { OutpostEKFParameters::kOutpostArmorCount };

    EKF ekf { EKF {} };
    Clock::time_point time_stamp;

    bool initialized { false };
    EKF::XVec pre_predict_x_ { EKF::XVec::Zero() };
    double last_predict_dt_s_ { 0.0 };
    bool has_predict_context_ { false };
    int update_count_ { 0 };

    const std::chrono::duration<double> reset_interval { 1.0 };
    OutpostState outpost_state {};
};

} // namespace rmcs::predictor

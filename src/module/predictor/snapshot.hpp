#pragma once

#include <eigen3/Eigen/Core>
#include <memory>
#include <variant>
#include <vector>

#include "module/predictor/outpost/ekf_parameter.hpp"
#include "utility/clock.hpp"
#include "utility/math/kalman_filter/ekf.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::predictor {

class Snapshot {
public:
    using NormalEKF  = util::EKF<11, 4>;
    using OutpostEKF = util::EKF<6, 4>;
    using Clock      = util::Clock;
    using State      = std::variant<NormalEKF::XVec, OutpostEKF::XVec>;

    struct Kinematics {
        Eigen::Vector3d center_position;
        double angular_velocity;
    };

    Snapshot(NormalEKF::XVec ekf_x, DeviceId device, CampColor color, int armor_num,
        Clock::time_point stamp) noexcept;
    Snapshot(OutpostEKF::XVec ekf_x, CampColor color, int armor_num, Clock::time_point stamp,
        int outpost_spin_sign, OutpostArmorLayout outpost_layout) noexcept;

    Snapshot(Snapshot const&);
    Snapshot(Snapshot&&) noexcept;
    Snapshot& operator=(Snapshot const&);
    Snapshot& operator=(Snapshot&&) noexcept;
    ~Snapshot() noexcept;

    auto state() const -> State;
    auto time_stamp() const -> Clock::time_point;
    auto predict_state_at(Clock::time_point t) const -> State;
    auto kinematics() const -> Kinematics;
    auto kinematics_at(Clock::time_point t) const -> Kinematics;

    auto predicted_armors(Clock::time_point t) const -> std::vector<Armor3D>;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rmcs::predictor

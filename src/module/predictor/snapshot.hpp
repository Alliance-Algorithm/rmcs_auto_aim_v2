#pragma once

#include <memory>

#include "utility/clock.hpp"
#include "utility/math/kalman_filter/ekf.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::predictor {

class Snapshot {
public:
    using EKF   = util::EKF<11, 4>;
    using Clock = util::Clock;

    Snapshot(EKF::XVec ekf_x, DeviceId device, CampColor color, int armor_num,
        Clock::time_point stamp) noexcept;
    Snapshot(Snapshot const&);
    Snapshot(Snapshot&&) noexcept;
    Snapshot& operator=(Snapshot const&);
    Snapshot& operator=(Snapshot&&) noexcept;
    ~Snapshot() noexcept;

    auto ekf_x() const -> EKF::XVec;

    auto time_stamp() const -> Clock::time_point;

    auto predict_at(Clock::time_point t) const -> EKF::XVec;
    auto predicted_armors(Clock::time_point t) const -> std::vector<Armor3D>;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rmcs::predictor

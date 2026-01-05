#pragma once

#include <chrono>
#include <memory>

#include "utility/math/kalman_filter/ekf.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::predictor {

class Snapshot {
public:
    using EKF       = util::EKF<11, 4>;
    using TimePoint = std::chrono::steady_clock::time_point;

    Snapshot(
        EKF::XVec ekf_x, DeviceId device, CampColor color, int armor_num, TimePoint stamp) noexcept;
    Snapshot(Snapshot const&);
    Snapshot(Snapshot&&) noexcept;
    Snapshot& operator=(Snapshot const&);
    Snapshot& operator=(Snapshot&&) noexcept;
    ~Snapshot() noexcept;

    auto ekf_x() const -> EKF::XVec;

    auto time_stamp() const -> TimePoint;

    auto predict_at(TimePoint t) const -> EKF::XVec;
    auto predicted_armors(TimePoint t) const -> std::vector<Armor3D>;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rmcs::predictor

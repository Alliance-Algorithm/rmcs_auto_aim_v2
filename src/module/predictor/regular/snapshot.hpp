#pragma once

#include "module/predictor/regular/ekf_parameter.hpp"
#include "module/predictor/snapshot.hpp"

#include <memory>
#include <vector>

namespace rmcs::predictor {

class RegularSnapshot {
public:
    using EKF = EKFParameters::EKF;

    explicit RegularSnapshot(EKF::XVec x, DeviceId device, CampColor color, int armor_num,
        TimePoint stamp);
    RegularSnapshot(RegularSnapshot const&) = delete;
    RegularSnapshot(RegularSnapshot&&) noexcept;
    RegularSnapshot& operator=(RegularSnapshot const&) = delete;
    RegularSnapshot& operator=(RegularSnapshot&&) noexcept;
    ~RegularSnapshot() noexcept;

    auto time_stamp() const -> TimePoint;
    auto device_id() const -> DeviceId;
    auto motion_at(TimePoint t) const -> TargetMotion;
    auto predicted_armors(TimePoint t) const -> std::vector<Armor3d>;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rmcs::predictor

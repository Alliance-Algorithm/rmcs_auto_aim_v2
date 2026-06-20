#pragma once

#include "module/predictor/outpost/armor_layout.hpp"
#include "module/predictor/outpost/ekf_parameter.hpp"
#include "module/predictor/snapshot.hpp"

#include <memory>
#include <vector>

namespace rmcs::predictor {

class OutpostSnapshot {
public:
    using EKF = OutpostEKFParameters::EKF;

    explicit OutpostSnapshot(EKF::XVec x, CampColor color, TimePoint stamp,
        OutpostArmorLayout layout, double angular_velocity);
    OutpostSnapshot(OutpostSnapshot const&) = delete;
    OutpostSnapshot(OutpostSnapshot&&) noexcept;
    OutpostSnapshot& operator=(OutpostSnapshot const&) = delete;
    OutpostSnapshot& operator=(OutpostSnapshot&&) noexcept;
    ~OutpostSnapshot() noexcept;

    auto time_stamp() const -> TimePoint;
    auto device_id() const -> DeviceId;
    auto motion_at(TimePoint t) const -> TargetMotion;
    auto predicted_armors(TimePoint t) const -> std::vector<Armor3d>;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rmcs::predictor

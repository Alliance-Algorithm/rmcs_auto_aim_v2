#pragma once

#include "module/predictor/snapshot.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::predictor::detail {

auto make_regular_snapshot(Snapshot::NormalEKF::XVec ekf_x, DeviceId device, CampColor color,
    int armor_num, Snapshot::Clock::time_point stamp) noexcept -> Snapshot;

} // namespace rmcs::predictor::detail

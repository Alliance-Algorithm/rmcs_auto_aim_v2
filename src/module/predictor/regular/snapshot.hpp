#pragma once

#include "module/predictor/snapshot.hpp"
#include "utility/math/kalman_filter/ekf.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::predictor::detail {

using NormalEKF  = util::EKF<11, 4>;
using OutpostEKF = util::EKF<6, 4>;

auto make_regular_snapshot(NormalEKF::XVec ekf_x, DeviceId device, CampColor color, int armor_num,
    TimePoint stamp) noexcept -> Snapshot;

} // namespace rmcs::predictor::detail

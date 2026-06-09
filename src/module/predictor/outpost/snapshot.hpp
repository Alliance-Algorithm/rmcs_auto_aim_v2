#pragma once

#include "module/predictor/outpost/armor_layout.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/math/kalman_filter/ekf.hpp"
#include "utility/robot/color.hpp"

namespace rmcs::predictor::detail {

using OutpostEKF = util::EKF<6, 4>;

auto make_outpost_snapshot(OutpostEKF::XVec ekf_x, CampColor color, int armor_num, TimePoint stamp,
    int outpost_spin_sign, OutpostArmorLayout outpost_layout) noexcept -> Snapshot;

} // namespace rmcs::predictor::detail

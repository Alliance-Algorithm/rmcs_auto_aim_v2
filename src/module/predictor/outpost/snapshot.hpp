#pragma once

#include "module/predictor/outpost/armor_layout.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/robot/color.hpp"

namespace rmcs::predictor::detail {

auto make_outpost_snapshot(Snapshot::OutpostEKF::XVec ekf_x, CampColor color, int armor_num,
    TimePoint stamp, int outpost_spin_sign, OutpostArmorLayout outpost_layout) noexcept -> Snapshot;

} // namespace rmcs::predictor::detail

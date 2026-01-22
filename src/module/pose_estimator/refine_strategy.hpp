#pragma once

#include "utility/math/angle.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::pose_estimator {

struct RefineStrategy {
    double yaw_range;    // Yaw 搜索半径 (弧度)
    double pitch_target; // Pitch 目标中心 (弧度)
    double pitch_range;  // Pitch 搜索半径 (弧度，为 0 则退化为固定约束)

    static auto choose_strategy(DeviceId genre, ArmorShape shape) -> RefineStrategy {
        if (genre == DeviceId::OUTPOST) {
            return { .yaw_range = util::deg2rad(60.0),
                .pitch_target   = util::deg2rad(15.0),
                .pitch_range    = 0.0 };
        }

        if (DeviceIds::kInfantry().contains(genre) && shape == ArmorShape::LARGE) {
            return { .yaw_range = util::deg2rad(45.0),
                .pitch_target   = 0.0,
                .pitch_range    = util::deg2rad(25.0) };
        }

        if (genre == DeviceId::HERO) {
            return { .yaw_range = util::deg2rad(30.0),
                .pitch_target   = util::deg2rad(-15.0),
                .pitch_range    = util::deg2rad(15.0) };
        }

        return { .yaw_range = util::deg2rad(30.0),
            .pitch_target   = util::deg2rad(-15.0),
            .pitch_range    = util::deg2rad(10.0) };
    }
};
}

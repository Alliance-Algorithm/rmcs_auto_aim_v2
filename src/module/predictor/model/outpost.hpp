#pragma once

#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs {

class OutpostModel {
    RMCS_PIMPL_DEFINITION(OutpostModel)

public:
    struct State {
        // 前哨站旋转中心在世界坐标系下的位置
        double x;
        double y;
        double z;

        // 以参考装甲板朝向为基准的角速度与 yaw 角
        double rotation_speed;
        double rotation_angle;
    };

    explicit OutpostModel(const Armor3d& armor) noexcept;

    auto predict(double dt) noexcept -> void;
    auto correct(const Armor3d& armor) noexcept -> void;
    auto state() noexcept -> State;
};

}

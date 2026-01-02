#pragma once
#include <chrono>

#include "ekf_parameter.hpp"
#include "utility/time.hpp"

namespace rmcs::predictor {
struct Snapshot {
    util::EKF<11, 4> ekf;
    DeviceId device;
    CampColor color;
    int armor_num;
    std::chrono::steady_clock::time_point stamp;

    auto predict_at(std::chrono::steady_clock::time_point t) const -> util::EKF<11, 4>::XVec {
        double dt = util::delta_time(t, stamp).count();
        // 使用 EKFParameters 里的 f(dt) 进行无副作用的外推
        return rmcs::predictor::EKFParameters::f(dt)(ekf.x);
    }
};
}

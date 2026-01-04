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

    auto predicted_armors(std::chrono::steady_clock::time_point t) const -> std::vector<Armor3D> {
        auto const& ekf_x = predict_at(t);
        auto _angle       = ekf_x[6];

        auto armors = std::vector<Armor3D> {};
        armors.reserve(armor_num);

        for (int id = 0; id < armor_num; ++id) {
            auto angle    = util::normalize_angle(_angle + id * 2 * CV_PI / armor_num);
            auto position = predictor::EKFParameters::h_armor_xyz(ekf_x, id, armor_num);

            auto armor        = Armor3D {};
            armor.genre       = device;
            armor.color       = camp_color2armor_color(color);
            armor.id          = id;
            armor.translation = position;
            armor.orientation = util::euler_to_quaternion(angle, 15. / 180 * CV_PI, 0);
            armors.emplace_back(armor);
        }

        return armors;
    }
};
}

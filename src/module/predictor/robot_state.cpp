#include "robot_state.hpp"
#include "module/predictor/ekf_parameter.hpp"
#include "utility/math/kalman_filter/ekf.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/id.hpp"
#include "utility/time.hpp"

using namespace rmcs::predictor;

struct RobotState::Impl {
    using EKF   = util::EKF<11, 4>;
    using Clock = std::chrono::steady_clock;
    using Stamp = Clock::time_point;

    explicit Impl(Armor3D const& armor, DeviceId const& device, Stamp const& t)
        : device(device)
        , armor_num(EKFParameters::armor_num(device))
        , time_stamp(t) {
        ekf = EKF { EKFParameters::x(armor), EKFParameters::P_initial_dig(device).asDiagonal() };
    }

    auto predict(Stamp const& t) -> void {
        auto dt = util::delta_time(t, time_stamp);
        ekf.predict(
            EKFParameters::f(dt), [dt](EKF::XVec const&) { return EKFParameters::F(dt); },
            EKFParameters::Q(device, dt));
        time_stamp = t;
    }

    auto update(Armor3D const& armor) -> void {
        int id = match_armors(ekf.x, armor);

        last_id = id;
        update_count++;

        auto const [pos_x, pos_y, pos_z] = armor.translation;
        auto const xyz                   = Eigen::Vector3d { pos_x, pos_y, pos_z };

        auto const& ypd = util::xyz2ypd(Eigen::Vector3d { pos_x, pos_y, pos_z });

        auto const [quat_x, quat_y, quat_z, quat_w] = armor.orientation;
        auto const& orientation = Eigen::Quaterniond { quat_w, quat_x, quat_y, quat_z };

        auto const& ypr = util::eulers(orientation);
        auto z          = EKF::ZVec {};
        z << ypd[0], ypd[1], ypd[2], ypr[0];

        ekf.update(
            z, [id, this](EKF::XVec const& x) { return EKFParameters::h(x, id, armor_num); },
            [id, this](EKF::XVec const& x) { return EKFParameters::H(x, id, armor_num); },
            EKFParameters::R(xyz, ypr, ypd), EKFParameters::x_add, EKFParameters::z_subtract);

        // 前哨站转速特判
        correct();
    }

    // x vx y vy z vz a w r l h
    constexpr auto is_convergened() -> bool {
        auto const r = ekf.x[8];
        auto const l = ekf.x[8] + ekf.x[9];

        auto const r_ok = (r > 0.05) && (r < 0.5);
        auto const l_ok = (l > 0.05) && (l < 0.5);

        if (r_ok && l_ok && update_count > 3) return true;

        if (r_ok && l_ok && device == DeviceId::OUTPOST && update_count > 10) return true;

        return false;
    }

    DeviceId device;
    int armor_num;

    EKF ekf;
    Stamp time_stamp;

    int last_id { 0 };
    int update_count { 0 };

    // 前哨站转速特判
    // x vx y vy z vz a w r l h
    constexpr auto correct() -> void {
        if (device == DeviceId::OUTPOST) {
            constexpr auto max_outpost_w = double { 2.51 };
            auto& w                      = ekf.x[7];
            if (std::abs(w) > 2.0) {
                w = w > 0 ? max_outpost_w : (-max_outpost_w);
            }
        }
    }

    constexpr auto calculate_armors(EKF::XVec const& x) const -> std::vector<Eigen::Vector4d> {
        auto armors = std::vector<Eigen::Vector4d> {};
        for (int i = 0; i < armor_num; i++) {
            auto angle = x[6];
            angle      = util::normalize_angle(angle + i * 2 * CV_PI / armor_num);

            auto xyz  = EKFParameters::h_armor_xyz(x, i, armor_num);
            auto xyza = Eigen::Vector4d { xyz[0], xyz[1], xyz[2], angle };
            armors.emplace_back(xyza);
        }
        return armors;
    }

    constexpr auto match_armors(EKF::XVec const& x, Armor3D const& armor) const -> int {
        auto armors_xyza = calculate_armors(x);

        auto const& [pos_x, pos_y, pos_z] = armor.translation;
        const auto xyz                    = Eigen::Vector3d { pos_x, pos_y, pos_z };
        const auto orientation  = Eigen::Quaterniond { armor.orientation.w, armor.orientation.x,
            armor.orientation.y, armor.orientation.z };
        const auto ypr_in_world = util::eulers(orientation);
        const auto ypd_in_world = util::xyz2ypd(xyz);

        struct Candidate {
            double distance;
            int id;
        };

        std::array<Candidate, 8> candidates;
        const auto n = std::min(static_cast<int>(candidates.size()), armor_num);

        for (int i = 0; i < n; ++i) {
            auto diff     = armors_xyza[i].head<3>() - Eigen::Vector3d::Zero();
            candidates[i] = { diff.squaredNorm(), i };
        }

        const int search_num = std::min(3, n);
        std::nth_element(candidates.begin(), candidates.begin() + search_num,
            candidates.begin() + n,
            [](auto const& a, auto const& b) { return a.distance < b.distance; });

        int best_id      = 0;
        double min_error = std::numeric_limits<double>::max();

        for (int i = 0; i < search_num; ++i) {
            auto const& [distance, index] = candidates[i];
            auto const& xyza              = armors_xyza[index];
            auto const& ypd_in_car        = util::xyz2ypd(xyza.head<3>());

            double yaw_error = std::abs(util::normalize_angle(ypr_in_world[0] - xyza[3]))
                + std::abs(util::normalize_angle(ypd_in_world[0] - ypd_in_car[0]));

            if (yaw_error < min_error) {
                min_error = yaw_error;
                best_id   = index;
            }
        }

        return best_id;
    }
};

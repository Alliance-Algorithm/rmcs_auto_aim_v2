#include "robot_state.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "module/predictor/regular/snapshot.hpp"
#include "utility/time.hpp"

using namespace rmcs::predictor;

struct RegularRobotState::Impl {
    struct MatchResult {
        int armor_id;
        double error;
        bool is_valid;
    };

    DeviceId device { DeviceId::UNKNOWN };
    CampColor color { CampColor::UNKNOWN };
    int armor_num { 0 };

    EKF ekf { EKF { } };
    TimePoint time_stamp;

    bool initialized { false };
    int update_count { 0 };

    const std::chrono::duration<double> reset_interval { 1.0 };
    const double angle_error_threshold { 0.65 };

    explicit Impl(TimePoint stamp) noexcept
        : time_stamp { stamp } { }

    auto initialize(Armor3D const& armor, TimePoint t) -> void {
        device         = armor.genre;
        color          = armor_color2camp_color(armor.color);
        armor_num      = EKFParameters::armor_num(armor.genre);
        time_stamp     = t;
        update_count   = 0;
        ekf = EKF { EKFParameters::x(armor), EKFParameters::P_initial_dig(device).asDiagonal() };
        initialized = true;
    }

    auto predict(TimePoint t) -> void {
        if (t <= time_stamp) return;

        if (initialized) {
            auto dt = util::delta_time(t, time_stamp);
            if (dt > reset_interval) {
                initialized  = false;
                update_count = 0;
                time_stamp   = t;
                return;
            }

            auto dt_s = dt.count();
            ekf.predict(
                EKFParameters::f(dt_s), [dt_s](EKF::XVec const&) { return EKFParameters::F(dt_s); },
                EKFParameters::Q(dt_s));
        }

        time_stamp = t;
    }

    auto update(std::span<Armor3D const> armors) -> bool {
        bool fused = false;
        for (auto const& armor : armors)
            fused = update_single(armor) || fused;

        if (fused) ++update_count;

        return fused;
    }

    auto is_converged() const -> bool {
        if (!initialized) return false;

        auto const r = ekf.x[8];
        auto const l = ekf.x[8] + ekf.x[9];

        auto const r_ok = (r > 0.05) && (r < 0.5);
        auto const l_ok = (l > 0.05) && (l < 0.5);

        int min_updates = 3;
        return r_ok && l_ok && update_count >= min_updates;
    }

    auto get_snapshot() const -> Snapshot {
        if (!initialized) return Snapshot::empty(time_stamp);
        return detail::make_regular_snapshot(ekf.x, device, color, armor_num, time_stamp);
    }

    auto distance() const -> double {
        if (!initialized) return std::numeric_limits<double>::infinity();
        return std::sqrt(ekf.x[0] * ekf.x[0] + ekf.x[2] * ekf.x[2]);
    }

private:
    auto match(Armor3D const& armor) const -> MatchResult {
        if (!initialized || armor.genre != device) return { -1, 1e10, false };

        auto armors_xyza = calculate_armors(ekf.x);

        auto const [pos_x, pos_y, pos_z] = armor.translation;
        auto const xyz                   = Eigen::Vector3d { pos_x, pos_y, pos_z };
        auto const orientation  = Eigen::Quaterniond { armor.orientation.w, armor.orientation.x,
            armor.orientation.y, armor.orientation.z };
        auto const ypr_in_world = util::eulers(orientation);
        auto const ypd_in_world = util::xyz2ypd(xyz);

        auto it =
            std::ranges::min_element(armors_xyza, [&](auto const& a_xyza, auto const& b_xyza) {
                auto get_error = [&](auto const& pred) {
                    auto ypd_pred = util::xyz2ypd(pred.template head<3>());
                    return std::abs(util::normalize_angle(ypr_in_world[0] - pred[3]))
                        + std::abs(util::normalize_angle(ypd_in_world[0] - ypd_pred[0]));
                };

                return get_error(a_xyza) < get_error(b_xyza);
            });

        auto const best_id   = static_cast<int>(std::distance(armors_xyza.begin(), it));
        auto const min_error = [&] {
            auto ypd_pred = util::xyz2ypd(it->template head<3>());
            return std::abs(util::normalize_angle(ypr_in_world[0] - (*it)[3]))
                + std::abs(util::normalize_angle(ypd_in_world[0] - ypd_pred[0]));
        }();

        return { best_id, min_error, min_error < angle_error_threshold };
    }

    auto update_single(Armor3D const& armor) -> bool {
        if (!initialized) {
            initialize(armor, time_stamp);
            return true;
        }
        if (armor.genre != device) return false;

        auto match_result = match(armor);
        if (!match_result.is_valid) return false;

        auto const [pos_x, pos_y, pos_z] = armor.translation;
        auto const xyz                   = Eigen::Vector3d { pos_x, pos_y, pos_z };
        auto const ypd                   = util::xyz2ypd(xyz);

        auto const [quat_x, quat_y, quat_z, quat_w] = armor.orientation;
        auto const orientation = Eigen::Quaterniond { quat_w, quat_x, quat_y, quat_z };
        auto const ypr         = util::eulers(orientation);

        auto z = EKF::ZVec { };
        z << ypd[0], ypd[1], ypd[2], ypr[0];

        ekf.update(
            z,
            [id = match_result.armor_id, this](
                EKF::XVec const& x) { return EKFParameters::h(device, x, id, armor_num); },
            [id = match_result.armor_id, this](
                EKF::XVec const& x) { return EKFParameters::H(device, x, id, armor_num); },
            EKFParameters::R(xyz, ypr, ypd), EKFParameters::x_add, EKFParameters::z_subtract);

        return true;
    }

    auto calculate_armors(EKF::XVec const& x) const -> std::vector<Eigen::Vector4d> {
        auto armors = std::vector<Eigen::Vector4d> { };
        armors.reserve(armor_num);
        for (int i = 0; i < armor_num; ++i) {
            auto angle = EKFParameters::armor_yaw(device, x, i);
            auto xyz   = EKFParameters::h_armor_xyz(device, x, i, armor_num);
            armors.emplace_back(xyz[0], xyz[1], xyz[2], angle);
        }
        return armors;
    }
};

RegularRobotState::RegularRobotState() noexcept
    : RegularRobotState(Clock::now()) { }

RegularRobotState::RegularRobotState(TimePoint stamp) noexcept
    : pimpl { std::make_unique<Impl>(stamp) } { }

RegularRobotState::~RegularRobotState() noexcept                                      = default;
RegularRobotState::RegularRobotState(RegularRobotState&&) noexcept                    = default;
auto RegularRobotState::operator=(RegularRobotState&&) noexcept -> RegularRobotState& = default;

auto RegularRobotState::initialize(Armor3D const& armor, TimePoint t) -> void {
    return pimpl->initialize(armor, t);
}

auto RegularRobotState::predict(TimePoint t) -> void { return pimpl->predict(t); }

auto RegularRobotState::update(std::span<Armor3D const> armors) -> bool {
    return pimpl->update(armors);
}

auto RegularRobotState::is_converged() const -> bool { return pimpl->is_converged(); }

auto RegularRobotState::get_snapshot() const -> Snapshot { return pimpl->get_snapshot(); }

auto RegularRobotState::distance() const -> double { return pimpl->distance(); }

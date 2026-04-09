#include "robot_state.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "module/predictor/ekf_parameter.hpp"
#include "module/predictor/outpost_ekf_parameter.hpp"
#include "module/predictor/outpost_state.hpp"
#include "utility/time.hpp"

using namespace rmcs::predictor;

struct RobotState::Impl {
    using EKF        = util::EKF<11, 4>;
    using OutpostEKF = OutpostEKFParameters::EKF;

    explicit Impl()
        : device { DeviceId::UNKNOWN }
        , color { CampColor::UNKNOWN }
        , armor_num { 0 }
        , ekf { EKF {} }
        , outpost_ekf { OutpostEKF {} }
        , time_stamp { Clock::now() }
        , initialized { false } { }

    auto initialize(Armor3D const& armor, Clock::time_point t) -> void {
        device     = armor.genre;
        color      = armor_color2camp_color(armor.color);
        armor_num  = EKFParameters::armor_num(armor.genre);
        time_stamp = t;
        if (device == DeviceId::OUTPOST) {
            outpost_ekf = OutpostEKF { OutpostEKFParameters::x(armor),
                OutpostEKFParameters::P_initial_dig().asDiagonal() };
            outpost_state.initialize(outpost_ekf.x);
        } else {
            ekf =
                EKF { EKFParameters::x(armor), EKFParameters::P_initial_dig(device).asDiagonal() };
            outpost_state.reset();
        }

        initialized = true;
    }

    auto get_snapshot() const -> Snapshot {
        if (device == DeviceId::OUTPOST) {
            auto outpost_order_idx = outpost_state.current_order_idx();
            auto outpost_spin_sign = outpost_state.current_spin_sign();
            return { outpost_ekf.x, color, armor_num, time_stamp, outpost_spin_sign,
                outpost_order_idx };
        }

        return { ekf.x, device, color, armor_num, time_stamp };
    }

    auto distance() const -> double {
        auto x = device == DeviceId::OUTPOST ? outpost_ekf.x[0] : ekf.x[0];
        auto y = device == DeviceId::OUTPOST ? outpost_ekf.x[2] : ekf.x[2];
        return std::sqrt(x * x + y * y);
    }

    auto predict(Clock::time_point t) -> void {
        if (initialized) {
            auto dt = util::delta_time(t, time_stamp);
            if (dt > reset_interval) {
                initialized  = false;
                update_count = 0;
                time_stamp   = t;
                outpost_state.reset();
                return;
            }

            auto dt_s = dt.count();
            if (device == DeviceId::OUTPOST) {
                auto spin_sign = outpost_state.current_spin_sign();
                outpost_state.record_predict_context(outpost_ekf.x, dt_s);
                outpost_ekf.predict(
                    OutpostEKFParameters::f(dt_s, spin_sign),
                    [dt_s](OutpostEKF::XVec const&) { return OutpostEKFParameters::F(dt_s); },
                    OutpostEKFParameters::Q(dt_s));
                outpost_state.correct(outpost_ekf.x);
            } else {
                ekf.predict(
                    EKFParameters::f(dt_s),
                    [dt_s](EKF::XVec const&) { return EKFParameters::F(dt_s); },
                    EKFParameters::Q(dt_s));
            }
        }

        time_stamp = t;
    }

    auto update(Armor3D const& armor) -> bool {
        if (!initialized) {
            initialize(armor, time_stamp);
            if (device == DeviceId::OUTPOST)
                return outpost_state.update(outpost_ekf,
                    std::span<Armor3D const> { &armor, static_cast<std::size_t>(1) }, armor_num,
                    update_count);
            return true;
        }

        if (device == DeviceId::OUTPOST)
            return update(std::span<Armor3D const> { &armor, static_cast<std::size_t>(1) });

        auto match_result = match(armor);
        if (!match_result.is_valid) return false;

        update_count++;

        auto const [pos_x, pos_y, pos_z] = armor.translation;
        auto const xyz                   = Eigen::Vector3d { pos_x, pos_y, pos_z };
        auto const ypd                   = util::xyz2ypd(xyz);

        auto const [quat_x, quat_y, quat_z, quat_w] = armor.orientation;
        auto const orientation = Eigen::Quaterniond { quat_w, quat_x, quat_y, quat_z };
        auto const ypr         = util::eulers(orientation);

        auto z = EKF::ZVec {};
        z << ypd[0], ypd[1], ypd[2], ypr[0];

        ekf.update(
            z,
            [id = match_result.armor_id, this](
                EKF::XVec const& x) { return EKFParameters::h(device, x, id, armor_num); },
            [id = match_result.armor_id, this](
                EKF::XVec const& x) { return EKFParameters::H(device, x, id, armor_num); },
            EKFParameters::R(xyz, ypr, ypd), EKFParameters::x_add, EKFParameters::z_subtract);

        correct();
        return true;
    }

    auto update(std::span<Armor3D const> armors) -> bool {
        if (armors.empty()) return false;
        auto used_armors = armors.first(
            std::min<std::size_t>(armors.size(), OutpostEKFParameters::kOutpostArmorCount));
        auto const incoming_outpost = used_armors.front().genre == DeviceId::OUTPOST;
        if ((device == DeviceId::OUTPOST || incoming_outpost) && !initialized)
            initialize(used_armors.front(), time_stamp);

        if (device != DeviceId::OUTPOST) {
            bool fused = false;
            for (auto const& armor : used_armors)
                fused = update(armor) || fused;
            return fused;
        }

        return outpost_state.update(outpost_ekf, used_armors, armor_num, update_count);
    }

    auto is_converged() const -> bool {
        if (device == DeviceId::OUTPOST)
            return outpost_state.is_converged(outpost_ekf.x, outpost_ekf.P(), update_count);

        auto const r = ekf.x[8];
        auto const l = ekf.x[8] + ekf.x[9];

        auto const r_ok = (r > 0.05) && (r < 0.5);
        auto const l_ok = (l > 0.05) && (l < 0.5);

        int min_updates = 3;
        if (r_ok && l_ok && update_count > min_updates) return true;

        return false;
    }

    DeviceId device;
    CampColor color;
    int armor_num;

    EKF ekf;
    OutpostEKF outpost_ekf;
    Clock::time_point time_stamp;

    bool initialized;
    int update_count { 0 };
    const std::chrono::duration<double> reset_interval { 1.0 };

    const double angle_error_threshold { 0.65 };
    OutpostState outpost_state;

    auto correct() -> void {
        if (device == DeviceId::OUTPOST) outpost_state.correct(outpost_ekf.x);
    }

    auto calculate_armors(EKF::XVec const& x) const -> std::vector<Eigen::Vector4d> {
        auto armors = std::vector<Eigen::Vector4d> {};
        for (int i = 0; i < armor_num; i++) {
            auto angle = EKFParameters::armor_yaw(device, x, i);
            auto xyz   = EKFParameters::h_armor_xyz(device, x, i, armor_num);
            auto xyza  = Eigen::Vector4d { xyz[0], xyz[1], xyz[2], angle };
            armors.emplace_back(xyza);
        }
        return armors;
    }

    auto match(Armor3D const& armor) const -> MatchResult {
        if (!initialized) return { -1, 1e10, false };
        if (device == DeviceId::OUTPOST) {
            auto match_result = outpost_state.match_armor(
                armor, OutpostState::MatchContext { outpost_ekf.x, outpost_ekf.P(), armor_num });
            return { match_result.armor_id, match_result.error, match_result.is_valid };
        }

        auto armors_xyza = calculate_armors(ekf.x);

        auto const [pos_x, pos_y, pos_z] = armor.translation;
        const auto xyz                   = Eigen::Vector3d { pos_x, pos_y, pos_z };
        const auto orientation  = Eigen::Quaterniond { armor.orientation.w, armor.orientation.x,
            armor.orientation.y, armor.orientation.z };
        const auto ypr_in_world = util::eulers(orientation);
        const auto ypd_in_world = util::xyz2ypd(xyz);

        auto it =
            std::ranges::min_element(armors_xyza, [&](auto const& a_xyza, auto const& b_xyza) {
                auto get_error = [&](auto const& pred) {
                    auto ypd_pred = util::xyz2ypd(pred.template head<3>());
                    return std::abs(util::normalize_angle(ypr_in_world[0] - pred[3]))
                        + std::abs(util::normalize_angle(ypd_in_world[0] - ypd_pred[0]));
                };

                return get_error(a_xyza) < get_error(b_xyza);
            });

        int best_id = static_cast<int>(std::distance(armors_xyza.begin(), it));

        auto min_error = [&](const auto& pred) {
            auto ypd_pred = util::xyz2ypd(pred.template head<3>());
            return std::abs(util::normalize_angle(ypr_in_world[0] - pred[3]))
                + std::abs(util::normalize_angle(ypd_in_world[0] - ypd_pred[0]));
        }(*it);

        return { best_id, min_error, (min_error < angle_error_threshold) };
    }
};

RobotState::RobotState() noexcept
    : pimpl { std::make_unique<Impl>() } { }
RobotState::~RobotState() noexcept = default;

auto RobotState::initialize(rmcs::Armor3D const& armor, Clock::time_point t) -> void {
    return pimpl->initialize(armor, t);
}

auto RobotState::predict(Clock::time_point t) -> void { return pimpl->predict(t); }

auto RobotState::match(Armor3D const& armor) const -> MatchResult { return pimpl->match(armor); }
auto RobotState::update(rmcs::Armor3D const& armor) -> bool { return pimpl->update(armor); }
auto RobotState::update(std::span<Armor3D const> armors) -> bool { return pimpl->update(armors); }

auto RobotState::is_converged() const -> bool { return pimpl->is_converged(); }

auto RobotState::get_snapshot() const -> Snapshot { return pimpl->get_snapshot(); }

auto RobotState::distance() const -> double { return pimpl->distance(); }

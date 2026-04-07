#include "robot_state.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <numeric>
#include <vector>

#include "module/predictor/ekf_parameter.hpp"
#include "module/predictor/outpost_state.hpp"
#include "utility/time.hpp"

using namespace rmcs::predictor;

struct RobotState::Impl {
    using EKF = util::EKF<11, 4>;

    explicit Impl()
        : device { DeviceId::UNKNOWN }
        , color { CampColor::UNKNOWN }
        , armor_num { 0 }
        , ekf { EKF {} }
        , time_stamp { Clock::now() }
        , initialized { false } { }

    auto initialize(Armor3D const& armor, Clock::time_point t) -> void {
        device    = armor.genre;
        color     = armor_color2camp_color(armor.color);
        armor_num = EKFParameters::armor_num(armor.genre);
        ekf = EKF { EKFParameters::x(armor), EKFParameters::P_initial_dig(device).asDiagonal() };
        time_stamp = t;
        if (device == DeviceId::OUTPOST) outpost_state.initialize(ekf.x);
        else outpost_state.reset();

        initialized = true;
    }

    auto get_snapshot() const -> Snapshot {
        auto outpost_order_idx = device == DeviceId::OUTPOST ? outpost_state.current_order_idx() : 0;
        return { ekf.x, device, color, armor_num, time_stamp, outpost_order_idx };
    }

    auto distance() const -> double {
        auto x = ekf.x[0], y = ekf.x[2];
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

            auto dt_s           = dt.count();
            if (device == DeviceId::OUTPOST) outpost_state.record_predict_context(ekf.x, dt_s);
            ekf.predict(
                EKFParameters::f(device, dt_s),
                [dt_s](EKF::XVec const&) { return EKFParameters::F(dt_s); },
                EKFParameters::Q(device, dt_s));
            if (device == DeviceId::OUTPOST) outpost_state.correct(ekf.x);
        }

        time_stamp = t;
    }

    auto update(Armor3D const& armor) -> bool {
        if (device == DeviceId::OUTPOST)
            return update(std::span<Armor3D const> { &armor, static_cast<std::size_t>(1) });

        if (!initialized) {
            initialize(armor, time_stamp);
            return true;
        }

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
            std::min<std::size_t>(armors.size(), EKFParameters::kOutpostArmorCount));
        if (device != DeviceId::OUTPOST) {
            bool fused = false;
            for (auto const& armor : used_armors)
                fused = update(armor) || fused;
            return fused;
        }

        if (!initialized) initialize(used_armors.front(), time_stamp);

        auto forced_spin_sign = outpost_state.spin_locked()
            ? std::optional<int> { outpost_state.current_spin_sign() }
            : std::nullopt;
        auto frame_match = outpost_state.match_frame(
            used_armors, OutpostState::MatchContext { ekf.x, ekf.P(), armor_num }, forced_spin_sign);
        if (!frame_match.valid) return false;

        update_count++;

        auto const allow_order_recalibration = used_armors.size() >= 2;
        ekf.x = outpost_state.build_hypothesis_state(ekf.x, frame_match.spin_sign);
        if (allow_order_recalibration && should_reseed_outpost_z_mid(used_armors.size()))
            ekf.x[4] = frame_match.z_mid;
        OutpostState::apply_constraints(ekf.x, frame_match.spin_sign);

        auto observation_order = std::vector<std::size_t>(used_armors.size());
        std::iota(observation_order.begin(), observation_order.end(), std::size_t { 0 });
        std::sort(observation_order.begin(), observation_order.end(),
            [&](std::size_t lhs, std::size_t rhs) {
                return frame_match.slots_by_observation[lhs]
                    < frame_match.slots_by_observation[rhs];
            });

        for (auto obs_index : observation_order) {
            auto const& armor = used_armors[obs_index];
            auto slot_idx     = frame_match.slots_by_observation[obs_index];

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
                [slot_idx, order_idx = frame_match.order_idx, this](EKF::XVec const& x) {
                    return EKFParameters::h(device, x, slot_idx, armor_num, order_idx);
                },
                [slot_idx, order_idx = frame_match.order_idx, this](EKF::XVec const& x) {
                    return EKFParameters::H(device, x, slot_idx, armor_num, order_idx);
                },
                EKFParameters::R(xyz, ypr, ypd), EKFParameters::x_add, EKFParameters::z_subtract);

            OutpostState::apply_constraints(ekf.x, frame_match.spin_sign);
        }

        outpost_state.apply_frame_match(frame_match, allow_order_recalibration);
        correct();
        return true;
    }

    auto is_converged() const -> bool {
        if (device == DeviceId::OUTPOST) return outpost_state.is_converged(ekf.x, update_count);

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
    Clock::time_point time_stamp;

    bool initialized;
    int update_count { 0 };
    const std::chrono::duration<double> reset_interval { 1.0 };

    const double angle_error_threshold { 0.65 };
    OutpostState outpost_state;

    auto should_reseed_outpost_z_mid(std::size_t observation_count) const -> bool {
        return outpost_state.should_reseed_z_mid(observation_count);
    }

    auto correct() -> void {
        if (device == DeviceId::OUTPOST) outpost_state.correct(ekf.x);
    }

    auto calculate_armors(EKF::XVec const& x) const -> std::vector<Eigen::Vector4d> {
        auto armors = std::vector<Eigen::Vector4d> {};
        for (int i = 0; i < armor_num; i++) {
            auto angle = EKFParameters::armor_yaw(device, x, i);
            auto outpost_order_idx = device == DeviceId::OUTPOST ? outpost_state.current_order_idx() : 0;
            auto xyz = EKFParameters::h_armor_xyz(device, x, i, armor_num, outpost_order_idx);
            auto xyza = Eigen::Vector4d { xyz[0], xyz[1], xyz[2], angle };
            armors.emplace_back(xyza);
        }
        return armors;
    }

    auto match(Armor3D const& armor) const -> MatchResult {
        if (!initialized) return { -1, 1e10, false };
        if (device == DeviceId::OUTPOST) {
            auto forced_spin_sign = outpost_state.spin_locked()
                ? std::optional<int> { outpost_state.current_spin_sign() }
                : std::nullopt;
            auto frame_match = outpost_state.match_frame(std::span<Armor3D const> { &armor, 1 },
                OutpostState::MatchContext { ekf.x, ekf.P(), armor_num }, forced_spin_sign);
            if (!frame_match.valid) return { -1, 1e10, false };
            return { frame_match.slots_by_observation.front(), frame_match.cost, true };
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

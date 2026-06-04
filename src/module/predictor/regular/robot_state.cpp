#include "robot_state.hpp"

#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

#include "utility/time.hpp"

using namespace rmcs::predictor;

namespace rmcs::predictor {
auto make_regular_snapshot(Snapshot::NormalEKF::XVec ekf_x, DeviceId device, CampColor color,
    int armor_num, TimePoint stamp) noexcept -> Snapshot;
}

struct RegularRobotState::Impl {
    struct MatchDecision {
        int matched_armor_id { kUnknownMatchedArmorId };
        double error { std::numeric_limits<double>::infinity() };
        bool is_valid { false };
    };

    struct BestMatch {
        std::size_t observation_index;
        MatchDecision decision;
    };

    DeviceId device { DeviceId::UNKNOWN };
    CampColor color { CampColor::UNKNOWN };
    int armor_num { 0 };

    EKF ekf { EKF {} };
    TimePoint time_stamp;

    bool initialized { false };
    int update_count { 0 };

    static constexpr int kUnknownMatchedArmorId = -1;
    int last_matched_armor_id { kUnknownMatchedArmorId };

    const std::chrono::duration<double> reset_interval { 1.0 };
    const double angle_error_threshold { 0.65 };
    const double visible_angle_threshold { std::numbers::pi / 2.0 };

    explicit Impl(TimePoint stamp) noexcept
        : time_stamp { stamp } { }

    auto initialize(Armor3D const& armor, TimePoint t) -> void {
        device                = armor.genre;
        color                 = armor_color2camp_color(armor.color);
        armor_num             = EKFParameters::armor_num(armor.genre);
        time_stamp            = t;
        update_count          = 0;
        last_matched_armor_id = 0;
        ekf = EKF { EKFParameters::x(armor), EKFParameters::P_initial_dig(device).asDiagonal() };
        initialized = true;
    }

    auto predict(TimePoint t) -> void {
        if (t <= time_stamp) return;

        if (initialized) {
            auto dt = util::delta_time(t, time_stamp);
            if (dt > reset_interval) {
                initialized           = false;
                update_count          = 0;
                last_matched_armor_id = kUnknownMatchedArmorId;
                time_stamp            = t;
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
        if (armors.empty()) return false;

        if (!initialized) {
            initialize(armors.front(), time_stamp);
            ++update_count;
            return true;
        }

        auto best_match = select_best_match(armors);
        if (!best_match.has_value()) return false;

        apply_match(armors, *best_match);
        ++update_count;

        return true;
    }

    auto is_converged() const -> bool {
        if (!initialized) return false;

        auto const r = ekf.x[8];
        auto const l = ekf.x[8] + ekf.x[9];

        auto const r_ok = (r > 0.05) && (r < 0.5);
        auto const l_ok = (l > 0.05) && (l < 0.5);

        int const min_updates = 3;
        return r_ok && l_ok && update_count >= min_updates;
    }

    auto get_snapshot() const -> Snapshot {
        if (!initialized) return Snapshot::empty(time_stamp);
        return make_regular_snapshot(ekf.x, device, color, armor_num, time_stamp);
    }

    auto distance() const -> double {
        if (!initialized) return std::numeric_limits<double>::infinity();
        return std::sqrt(ekf.x[0] * ekf.x[0] + ekf.x[2] * ekf.x[2]);
    }

private:
    auto decide_match(Armor3D const& armor) const -> MatchDecision {
        if (!initialized || armor.genre != device) return {};

        auto armors_xyza = calculate_armors(ekf.x);

        auto const [pos_x, pos_y, pos_z] = armor.translation;
        auto const xyz                   = Eigen::Vector3d { pos_x, pos_y, pos_z };
        auto const orientation  = Eigen::Quaterniond { armor.orientation.w, armor.orientation.x,
            armor.orientation.y, armor.orientation.z };
        auto const ypr_in_world = util::eulers(orientation);
        auto const ypd_in_world = util::xyz2ypd(xyz);

        auto best_matched_armor_id           = kUnknownMatchedArmorId;
        auto min_error                       = std::numeric_limits<double>::infinity();
        auto const opposite_matched_armor_id = [&] {
            if (armor_num != 4 || last_matched_armor_id == kUnknownMatchedArmorId) {
                return kUnknownMatchedArmorId;
            }
            return (last_matched_armor_id + armor_num / 2) % armor_num;
        }();

        for (auto&& [candidate_armor_id, pred] : armors_xyza | std::views::enumerate) {
            if (candidate_armor_id == opposite_matched_armor_id) continue;

            auto const ypd_pred   = util::xyz2ypd(pred.template head<3>());
            auto const view_delta = std::abs(util::normalize_angle(pred[3] - ypd_pred[0]));
            if (view_delta > visible_angle_threshold) continue;

            auto const error = std::abs(util::normalize_angle(ypr_in_world[0] - pred[3]))
                + std::abs(util::normalize_angle(ypd_in_world[0] - ypd_pred[0]));
            if (error >= min_error) continue;

            best_matched_armor_id = candidate_armor_id;
            min_error             = error;
        }

        if (best_matched_armor_id == kUnknownMatchedArmorId) return {};

        return {
            .matched_armor_id = best_matched_armor_id,
            .error            = min_error,
            .is_valid         = min_error < angle_error_threshold,
        };
    }

    auto select_best_match(std::span<Armor3D const> armors) const -> std::optional<BestMatch> {
        auto best_match = std::optional<BestMatch> {};
        for (std::size_t observation_index = 0; observation_index < armors.size();
            ++observation_index) {
            auto decision = decide_match(armors[observation_index]);
            if (!decision.is_valid) continue;
            if (best_match.has_value() && decision.error >= best_match->decision.error) continue;

            best_match = { .observation_index = observation_index, .decision = decision };
        }

        return best_match;
    }

    auto apply_match(std::span<Armor3D const> armors, BestMatch const& best_match) -> void {
        auto const& armor    = armors[best_match.observation_index];
        auto const& decision = best_match.decision;

        if (armor.genre != device) return;

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
            [id = decision.matched_armor_id, this](
                EKF::XVec const& x) { return EKFParameters::h(device, x, id, armor_num); },
            [id = decision.matched_armor_id, this](
                EKF::XVec const& x) { return EKFParameters::H(device, x, id, armor_num); },
            EKFParameters::R(xyz, ypr, ypd), EKFParameters::x_add, EKFParameters::z_subtract);

        last_matched_armor_id = decision.matched_armor_id;
    }

    auto calculate_armors(EKF::XVec const& x) const -> std::vector<Eigen::Vector4d> {
        auto armors = std::vector<Eigen::Vector4d> {};
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

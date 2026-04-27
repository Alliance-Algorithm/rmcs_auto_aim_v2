#include "robot_state.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "module/predictor/regular/snapshot.hpp"
#include "utility/math/mahalanobis.hpp"
#include "utility/time.hpp"

using namespace rmcs::predictor;

struct RegularRobotState::Impl {

    struct MatchResult {
        int armor_id;
        double error;
        bool is_valid;
    };

    static constexpr auto kResetInterval       = std::chrono::duration<double> { 1.0 };
    static constexpr auto kAngleErrorThreshold = double { 0.35 };
    static constexpr auto kChi2Gate           = double { 13.277 };

    DeviceId device { DeviceId::UNKNOWN };
    CampColor color { CampColor::UNKNOWN };
    int armor_num { 0 };

    EKF ekf { EKF { } };
    TimePoint time_stamp;

    bool initialized { false };
    int update_count { 0 };
    int nis_fail_count { 0 };

    explicit Impl(TimePoint stamp) noexcept
        : time_stamp { stamp } { }

    auto initialize(Armor3D const& armor, TimePoint t) -> void {
        device         = armor.genre;
        color          = armor_color2camp_color(armor.color);
        armor_num      = EKFParameters::armor_num(armor.genre);
        time_stamp     = t;
        update_count   = 0;
        nis_fail_count = 0;
        ekf = EKF { EKFParameters::x(armor), EKFParameters::P_initial_dig(device).asDiagonal() };
        initialized = true;
    }

    auto predict(TimePoint t) -> void {
        if (t <= time_stamp) return;

        if (initialized) {
            auto dt = util::delta_time(t, time_stamp);
            if (dt > kResetInterval) {
                initialized    = false;
                update_count   = 0;
                nis_fail_count = 0;
                time_stamp     = t;
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

        auto get_error = [&](auto const& pred) {
            auto ypd_pred = util::xyz2ypd(pred.template head<3>());
            return std::abs(util::normalize_angle(ypr_in_world[0] - pred[3]))
                + std::abs(util::normalize_angle(ypd_in_world[0] - ypd_pred[0]));
        };

        auto ranked = std::vector<std::pair<int, double>> { };
        ranked.reserve(armors_xyza.size());
        for (int id = 0; id < static_cast<int>(armors_xyza.size()); ++id)
            ranked.emplace_back(id, (armors_xyza[id].template head<3>() - xyz).norm());
        std::ranges::sort(ranked, { }, &std::pair<int, double>::second);

        auto const kMaxCandidates { 3 };
        auto best_id    = int { -1 };
        auto best_error = double { std::numeric_limits<double>::max() };

        for (auto id = 0; id < std::min(kMaxCandidates, static_cast<int>(ranked.size())); ++id) {
            auto candidate_id    = ranked[id].first;
            auto candidate_error = get_error(armors_xyza[candidate_id]);
            if (candidate_error < best_error) {
                best_error = candidate_error;
                best_id    = candidate_id;
            }
        }

        return { best_id, best_error, best_error < kAngleErrorThreshold };
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

        auto id = match_result.armor_id;
        auto z  = EKF::ZVec { };
        z << ypd[0], ypd[1], ypd[2], ypr[0];

        auto r      = EKFParameters::R(xyz, ypr, ypd);
        auto z_pred = EKFParameters::h(device, ekf.x, id, armor_num);
        auto h_jac  = EKFParameters::H(device, ekf.x, id, armor_num);
        auto y      = EKFParameters::z_subtract(z, z_pred);
        auto s      = h_jac * ekf.P() * h_jac.transpose() + r;
        auto nis    = util::mahalanobis_distance(y, s);
        if (!nis.has_value() || *nis > kChi2Gate) {
            ++nis_fail_count;
            return false;
        }

        auto x_pre  = ekf.x;
        auto P_pre  = ekf.P();

        ekf.update(
            z,
            [id, this](EKF::XVec const& x) { return EKFParameters::h(device, x, id, armor_num); },
            [id, this](EKF::XVec const& x) { return EKFParameters::H(device, x, id, armor_num); },
            r, EKFParameters::x_add, EKFParameters::z_subtract);

        auto const r_ok = ekf.x[8] > 0.05 && ekf.x[8] < 0.5;
        auto const l_ok =
            (ekf.x[8] + ekf.x[9]) > 0.05 && (ekf.x[8] + ekf.x[9]) < 0.5;
        if (!r_ok || !l_ok) {
            ekf.x  = x_pre;
            ekf.P() = P_pre;
            ++nis_fail_count;
            return false;
        }

        nis_fail_count = 0;
        return true;
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

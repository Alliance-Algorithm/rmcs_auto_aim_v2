#include "robot_state.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <span>
#include <vector>

#include "module/predictor/ekf_parameter.hpp"
#include "utility/time.hpp"

using namespace rmcs::predictor;

struct RobotState::Impl {
    using EKF                                    = util::EKF<11, 4>;
    static constexpr int kUnknownOutpostOrderIdx = -1;
    static constexpr int kUnknownOutpostSlotIdx  = -1;

    struct OutpostObservation {
        Eigen::Vector3d xyz;
        Eigen::Vector3d ypr;
        Eigen::Vector3d ypd;
        EKF::ZVec z;
        EKF::RMat R;
    };

    struct OutpostFrameMatch {
        bool valid { false };
        int order_idx { 0 };
        int spin_sign { +1 };
        double cost { 1e10 };
        double z_mid { 0.0 };
        int visible_slot_idx { kUnknownOutpostSlotIdx };
        std::vector<int> slots_by_observation;
    };

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
        reset_outpost_hypothesis();
        if (device == DeviceId::OUTPOST) {
            outpost_active_order_idx = neutral_outpost_order_for_slot(0);
            apply_outpost_constraints(ekf.x, outpost_spin_sign);
        }

        initialized = true;
    }

    auto get_snapshot() const -> Snapshot {
        return { ekf.x, device, color, armor_num, time_stamp, current_outpost_order_idx() };
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
                reset_outpost_hypothesis();
                return;
            }

            auto dt_s           = dt.count();
            pre_predict_x_      = ekf.x;
            last_predict_dt_s_  = dt_s;
            has_predict_context = true;
            ekf.predict(
                EKFParameters::f(device, dt_s),
                [dt_s](EKF::XVec const&) { return EKFParameters::F(dt_s); },
                EKFParameters::Q(device, dt_s));
            if (device == DeviceId::OUTPOST)
                apply_outpost_constraints(ekf.x, current_outpost_spin_sign());
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
        auto used_armors = armors.first(std::min<std::size_t>(armors.size(), kOutpostArmorCount));
        if (device != DeviceId::OUTPOST) {
            bool fused = false;
            for (auto const& armor : used_armors)
                fused = update(armor) || fused;
            return fused;
        }

        if (!initialized) initialize(used_armors.front(), time_stamp);

        auto frame_match = outpost_spin_locked ? match_outpost_frame(used_armors, outpost_spin_sign)
                                               : match_outpost_frame(used_armors);
        if (!frame_match.valid) return false;

        update_count++;

        auto const allow_order_recalibration = used_armors.size() >= 2;
        if (allow_order_recalibration) outpost_active_order_idx = frame_match.order_idx;
        ekf.x = build_outpost_hypothesis_state(frame_match.spin_sign);
        if (allow_order_recalibration && should_reseed_outpost_z_mid(used_armors.size()))
            ekf.x[4] = frame_match.z_mid;
        apply_outpost_constraints(ekf.x, frame_match.spin_sign);

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

            apply_outpost_constraints(ekf.x, frame_match.spin_sign);
        }

        if (outpost_spin_locked) outpost_spin_sign = frame_match.spin_sign;
        else observe_outpost_spin_candidate(frame_match.spin_sign);
        last_visible_slot_idx = frame_match.visible_slot_idx;
        if (allow_order_recalibration) commit_outpost_order(frame_match.order_idx);
        correct();
        return true;
    }

    auto is_converged() const -> bool {
        if (device == DeviceId::OUTPOST) {
            auto const r     = ekf.x[8];
            auto const dz    = ekf.x[9];
            auto const r_ok  = std::abs(r - kOutpostRadius) < 1e-2;
            auto const dz_ok = std::abs(dz - kOutpostArmorHeightStep) < 1e-2;
            return r_ok && dz_ok && update_count > 10;
        }

        auto const r = ekf.x[8];
        auto const l = ekf.x[8] + ekf.x[9];

        auto const r_ok = (r > 0.05) && (r < 0.5);
        auto const l_ok = (l > 0.05) && (l < 0.5);

        int min_updates = (device == DeviceId::OUTPOST) ? 10 : 3;
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
    const double outpost_yaw_gate { util::deg2rad(35.) };
    const double outpost_azimuth_gate { util::deg2rad(35.) };
    const double outpost_z_gate { 0.06 };
    const double outpost_single_observation_z_gate { 0.18 };
    const double outpost_single_observation_z_weight { 0.15 };
    const double outpost_single_observation_cost_gate { 3.0 };
    const double outpost_mahalanobis_gate { 20.0 };
    const int outpost_spin_confirm_frames { 3 };

    int outpost_order_idx { kUnknownOutpostOrderIdx };
    int outpost_active_order_idx { kUnknownOutpostOrderIdx };
    int last_outpost_order_idx { kUnknownOutpostOrderIdx };
    int outpost_order_streak { 0 };

    int outpost_spin_sign { +1 };
    bool outpost_spin_locked { false };
    int outpost_last_spin_candidate { 0 };
    int outpost_spin_candidate_streak { 0 };
    int last_visible_slot_idx { kUnknownOutpostSlotIdx };

    EKF::XVec pre_predict_x_ { EKF::XVec::Zero() };
    double last_predict_dt_s_ { 0.0 };
    bool has_predict_context { false };
    auto reset_outpost_hypothesis() -> void {
        outpost_order_idx             = kUnknownOutpostOrderIdx;
        outpost_active_order_idx      = kUnknownOutpostOrderIdx;
        last_outpost_order_idx        = kUnknownOutpostOrderIdx;
        outpost_order_streak          = 0;
        outpost_spin_sign             = +1;
        outpost_spin_locked           = false;
        outpost_last_spin_candidate   = 0;
        outpost_spin_candidate_streak = 0;
        last_visible_slot_idx         = kUnknownOutpostSlotIdx;
        pre_predict_x_                = EKF::XVec::Zero();
        last_predict_dt_s_            = 0.0;
        has_predict_context           = false;
    }

    auto current_outpost_order_idx() const -> int {
        if (device != DeviceId::OUTPOST) return 0;
        if (outpost_order_idx != kUnknownOutpostOrderIdx) return outpost_order_idx;
        return neutral_outpost_order_for_slot(0);
    }

    auto current_outpost_spin_sign() const -> int {
        if (device != DeviceId::OUTPOST) return +1;
        return outpost_spin_sign;
    }

    auto observe_outpost_spin_candidate(int candidate_sign) -> void {
        if (device != DeviceId::OUTPOST) return;

        auto normalized_sign = candidate_sign >= 0 ? +1 : -1;
        outpost_spin_sign    = normalized_sign;

        if (normalized_sign == outpost_last_spin_candidate) outpost_spin_candidate_streak++;
        else {
            outpost_last_spin_candidate   = normalized_sign;
            outpost_spin_candidate_streak = 1;
        }

        if (outpost_spin_candidate_streak >= outpost_spin_confirm_frames) {
            outpost_spin_sign             = normalized_sign;
            outpost_spin_locked           = true;
            outpost_last_spin_candidate   = normalized_sign;
            outpost_spin_candidate_streak = outpost_spin_confirm_frames;
        }
    }

    static auto apply_outpost_constraints(EKF::XVec& x, int spin_sign) -> void {
        x[6]  = util::normalize_angle(x[6]);
        x[5]  = 0.;
        x[7]  = spin_sign >= 0 ? kOutpostAngularSpeed : -kOutpostAngularSpeed;
        x[8]  = kOutpostRadius;
        x[9]  = kOutpostArmorHeightStep;
        x[10] = 0.;
    }

    static auto neutral_outpost_order_for_slot(int slot_idx) -> int {
        for (int order_idx = 0; order_idx < kOutpostHeightOrderCount; ++order_idx) {
            if (EKFParameters::outpost_height_rank(order_idx, slot_idx) == 0) return order_idx;
        }
        return 0;
    }

    static auto advance_outpost_slot(int slot_idx, int spin_sign) -> int {
        auto normalized_slot =
            (slot_idx % kOutpostArmorCount + kOutpostArmorCount) % kOutpostArmorCount;
        auto step = spin_sign >= 0 ? -1 : +1;
        return (normalized_slot + step + kOutpostArmorCount) % kOutpostArmorCount;
    }

    static auto select_visible_slot(std::vector<OutpostObservation> const& observations,
        std::vector<int> const& assignment) -> int {
        auto best_slot        = kUnknownOutpostSlotIdx;
        auto best_abs_azimuth = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < observations.size(); ++i) {
            if (assignment[i] < 0) continue;
            auto abs_azimuth = std::abs(observations[i].ypd[0]);
            if (abs_azimuth >= best_abs_azimuth) continue;
            best_abs_azimuth = abs_azimuth;
            best_slot        = assignment[i];
        }
        return best_slot;
    }

    auto has_committed_outpost_order() const -> bool {
        return outpost_order_idx != kUnknownOutpostOrderIdx;
    }

    auto should_reseed_outpost_z_mid(std::size_t observation_count) const -> bool {
        return observation_count >= 2;
    }

    auto build_outpost_hypothesis_state(int spin_sign) const -> EKF::XVec {
        auto x_test = has_predict_context ? pre_predict_x_ : ekf.x;
        apply_outpost_constraints(x_test, spin_sign);
        if (has_predict_context && last_predict_dt_s_ > 0.) {
            x_test = EKFParameters::f(DeviceId::OUTPOST, last_predict_dt_s_)(x_test);
            apply_outpost_constraints(x_test, spin_sign);
        }
        return x_test;
    }

    auto commit_outpost_order(int candidate_order_idx) -> void {
        if (candidate_order_idx == outpost_order_idx) {
            last_outpost_order_idx = candidate_order_idx;
            outpost_order_streak   = std::min(outpost_order_streak + 1, 1024);
            return;
        }

        if (candidate_order_idx == last_outpost_order_idx) outpost_order_streak++;
        else {
            last_outpost_order_idx = candidate_order_idx;
            outpost_order_streak   = 1;
        }

        if (outpost_order_streak >= 2) {
            outpost_order_idx      = candidate_order_idx;
            last_outpost_order_idx = candidate_order_idx;
            outpost_order_streak   = 0;
        }
    }

    auto correct() -> void {
        if (device == DeviceId::OUTPOST)
            apply_outpost_constraints(ekf.x, current_outpost_spin_sign());
    }

    auto calculate_armors(EKF::XVec const& x) const -> std::vector<Eigen::Vector4d> {
        auto armors = std::vector<Eigen::Vector4d> {};
        for (int i = 0; i < armor_num; i++) {
            auto angle = EKFParameters::armor_yaw(device, x, i, armor_num);
            auto xyz =
                EKFParameters::h_armor_xyz(device, x, i, armor_num, current_outpost_order_idx());
            auto xyza = Eigen::Vector4d { xyz[0], xyz[1], xyz[2], angle };
            armors.emplace_back(xyza);
        }
        return armors;
    }

    static auto outpost_observation(Armor3D const& armor) -> OutpostObservation {
        auto const [pos_x, pos_y, pos_z] = armor.translation;
        auto const xyz                   = Eigen::Vector3d { pos_x, pos_y, pos_z };

        auto const [quat_x, quat_y, quat_z, quat_w] = armor.orientation;
        auto const orientation = Eigen::Quaterniond { quat_w, quat_x, quat_y, quat_z };

        auto const ypr = util::eulers(orientation);
        auto const ypd = util::xyz2ypd(xyz);

        auto z = EKF::ZVec {};
        z << ypd[0], ypd[1], ypd[2], ypr[0];

        return { xyz, ypr, ypd, z, EKFParameters::R(xyz, ypr, ypd) };
    }

    auto match(Armor3D const& armor) const -> MatchResult {
        if (!initialized) return { -1, 1e10, false };
        if (device == DeviceId::OUTPOST) {
            auto frame_match = outpost_spin_locked
                ? match_outpost_frame(std::span<Armor3D const> { &armor, 1 }, outpost_spin_sign)
                : match_outpost_frame(std::span<Armor3D const> { &armor, 1 });
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

    auto match_outpost_frame(std::span<Armor3D const> armors,
        std::optional<int> forced_spin_sign = std::nullopt) const -> OutpostFrameMatch {
        if (armors.empty()) return {};

        auto observations = std::vector<OutpostObservation> {};
        observations.reserve(std::min<std::size_t>(armors.size(), kOutpostArmorCount));
        for (auto const& armor : armors) {
            if (observations.size() == static_cast<std::size_t>(kOutpostArmorCount)) break;
            observations.emplace_back(outpost_observation(armor));
        }

        auto best = OutpostFrameMatch {};
        best.cost = std::numeric_limits<double>::infinity();

        auto assignment = std::vector<int>(observations.size(), -1);
        auto slot_used  = std::array<bool, kOutpostArmorCount> { false, false, false };
        auto const single_observation    = observations.size() == 1;
        auto const force_committed_order = single_observation && has_committed_outpost_order();
        auto const continuity_spin_sign  = forced_spin_sign.has_value()
             ? (forced_spin_sign.value() >= 0 ? +1 : -1)
             : current_outpost_spin_sign();

        auto evaluate_assignment = [&](int order_idx, int spin_sign) {
            auto x_test = build_outpost_hypothesis_state(spin_sign);
            if (!single_observation) {
                double z_sum = 0.0;
                for (std::size_t i = 0; i < observations.size(); ++i) {
                    auto slot = assignment[i];
                    z_sum += observations[i].xyz[2]
                        - EKFParameters::outpost_height_rank(order_idx, slot) * x_test[9];
                }
                x_test[4] = z_sum / static_cast<double>(observations.size());
            }

            double total_cost = 0.0;
            for (std::size_t i = 0; i < observations.size(); ++i) {
                auto slot       = assignment[i];
                auto const& obs = observations[i];
                auto xyz_pred =
                    EKFParameters::h_armor_xyz(device, x_test, slot, armor_num, order_idx);
                auto ypd_pred      = util::xyz2ypd(xyz_pred);
                auto yaw_pred      = EKFParameters::armor_yaw(device, x_test, slot, armor_num);
                auto yaw_error     = std::abs(util::normalize_angle(obs.ypr[0] - yaw_pred));
                auto azimuth_error = std::abs(util::normalize_angle(obs.ypd[0] - ypd_pred[0]));
                auto z_error       = std::abs(obs.xyz[2] - xyz_pred[2]);
                auto const z_gate =
                    single_observation ? outpost_single_observation_z_gate : outpost_z_gate;
                if (yaw_error > outpost_yaw_gate || azimuth_error > outpost_azimuth_gate
                    || z_error > z_gate)
                    return;

                double cost = 0.0;
                if (single_observation) {
                    auto yaw_cost     = yaw_error / outpost_yaw_gate;
                    auto azimuth_cost = azimuth_error / outpost_azimuth_gate;
                    auto z_cost       = z_error / outpost_single_observation_z_gate;
                    cost              = yaw_cost * yaw_cost + azimuth_cost * azimuth_cost
                        + outpost_single_observation_z_weight * z_cost * z_cost;
                } else {
                    auto H          = EKFParameters::H(device, x_test, slot, armor_num, order_idx);
                    auto z_hat      = EKFParameters::h(device, x_test, slot, armor_num, order_idx);
                    auto innovation = EKFParameters::z_subtract(obs.z, z_hat);
                    auto S          = H * ekf.P() * H.transpose() + obs.R;
                    auto solved     = S.ldlt().solve(innovation);
                    cost            = innovation.dot(solved);
                }
                if (!std::isfinite(cost)) return;
                total_cost += cost;
            }

            if (total_cost < best.cost) {
                best.valid                = true;
                best.order_idx            = order_idx;
                best.spin_sign            = spin_sign;
                best.cost                 = total_cost;
                best.z_mid                = x_test[4];
                best.visible_slot_idx     = select_visible_slot(observations, assignment);
                best.slots_by_observation = assignment;
            }
        };

        auto search = [&](auto&& self, std::size_t obs_index, int order_idx,
                          int spin_sign) -> void {
            if (obs_index == observations.size()) {
                evaluate_assignment(order_idx, spin_sign);
                return;
            }

            for (int slot = 0; slot < armor_num; ++slot) {
                if (slot_used[slot]) continue;
                if (single_observation && last_visible_slot_idx != kUnknownOutpostSlotIdx) {
                    auto reachable_slot =
                        advance_outpost_slot(last_visible_slot_idx, continuity_spin_sign);
                    if (slot != last_visible_slot_idx && slot != reachable_slot) continue;
                }
                if (!force_committed_order && single_observation && !has_committed_outpost_order()
                    && EKFParameters::outpost_height_rank(order_idx, slot) != 0)
                    continue;
                slot_used[slot]       = true;
                assignment[obs_index] = slot;
                self(self, obs_index + 1, order_idx, spin_sign);
                slot_used[slot]       = false;
                assignment[obs_index] = -1;
            }
        };

        auto search_order = [&](int order_idx) {
            if (forced_spin_sign.has_value()) {
                auto spin_sign = forced_spin_sign.value() >= 0 ? +1 : -1;
                search(search, 0, order_idx, spin_sign);
                return;
            }

            for (int spin_sign : { -1, +1 }) {
                search(search, 0, order_idx, spin_sign);
            }
        };

        if (force_committed_order) {
            search_order(outpost_order_idx);
        } else {
            for (int order_idx = 0; order_idx < kOutpostHeightOrderCount; ++order_idx) {
                search_order(order_idx);
            }
        }

        auto const max_cost =
            single_observation ? outpost_single_observation_cost_gate : outpost_mahalanobis_gate;
        if (!best.valid || best.cost > max_cost) return {};
        return best;
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

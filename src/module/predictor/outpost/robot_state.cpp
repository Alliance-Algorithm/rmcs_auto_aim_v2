#include "robot_state.hpp"

#include "utility/math/mahalanobis.hpp"
#include "utility/time.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <span>

namespace rmcs::predictor {

struct OutpostRobotState::Impl {
    explicit Impl(TimePoint stamp) noexcept
        : time_stamp { stamp } { }

    auto initialize(Armor3d const& armor, TimePoint t) -> void {
        color      = armor_color2camp_color(armor.color);
        ekf        = EKF { OutpostEKFParameters::x(armor),
            OutpostEKFParameters::P_initial_dig().asDiagonal() };
        time_stamp = t;

        layout       = OutpostArmorLayout {};
        update_count = 0;
        initialized  = true;
    }

    auto predict(TimePoint t) -> void {
        if (t <= time_stamp) return;

        if (initialized) {
            auto dt = rmcs::util::delta_time(t, time_stamp);
            if (dt > reset_interval) {
                reset_runtime_state(t);
                return;
            }

            auto const dt_s = dt.count();
            ekf.predict(
                OutpostEKFParameters::f(dt_s),
                [dt_s](EKF::XVec const&) { return OutpostEKFParameters::F(dt_s); },
                OutpostEKFParameters::Q(dt_s));
        }

        time_stamp = t;
    }

    auto update(std::span<Armor3d const> armors) -> bool {
        if (armors.empty()) return false;

        if (!initialized) {
            initialize(armors.front(), time_stamp);
            return true;
        }

        auto best_match = select_best_match(armors);
        if (!best_match.has_value()) return false;

        apply_match(*best_match);
        return true;
    }

    // TODO:添加收敛条件
    auto is_converged() const -> bool {
        if (!initialized) return false;
        return true;
    }

    auto get_snapshot() const -> Snapshot {
        if (!initialized) return Snapshot::empty(time_stamp);
        return Snapshot::make_outpost(ekf.x, color, time_stamp, layout);
    }

    auto distance() const -> double {
        return initialized ? std::sqrt(ekf.x[0] * ekf.x[0] + ekf.x[2] * ekf.x[2])
                           : std::numeric_limits<double>::infinity();
    }

private:
    struct MatchedArmor {
        Armor3d armor;
        double height_offset;
        OutpostArmorLayout layout;
    };

    auto resolve_layout(int armor_id, double height_offset) const -> OutpostArmorLayout {
        auto next_layout = layout;
        if (next_layout.height_template.has_value() || armor_id == 0) return next_layout;

        auto const positive_on_slot1 = (armor_id == 1) == (height_offset > 0.0);
        next_layout.height_template  = positive_on_slot1
             ? OutpostArmorLayout::HeightTemplate::PositiveOnSlot1
             : OutpostArmorLayout::HeightTemplate::NegativeOnSlot1;
        return next_layout;
    }

    auto visit_match_hypotheses(auto&& visit) const -> void {
        using rmcs::kOutpostArmorHeightStep;

        auto const visit_hypothesis = [&](int armor_id, double height_offset = 0.0) {
            visit(armor_id, height_offset, resolve_layout(armor_id, height_offset));
        };

        visit_hypothesis(0);

        if (!layout.height_template.has_value()) {
            auto const visit_side_hypotheses = [&](int armor_id) {
                visit_hypothesis(armor_id, +kOutpostArmorHeightStep);
                visit_hypothesis(armor_id, -kOutpostArmorHeightStep);
            };

            visit_side_hypotheses(1);
            visit_side_hypotheses(2);
            return;
        }

        for (int armor_id : { 1, 2 }) {
            visit_hypothesis(
                armor_id, OutpostEKFParameters::height_offset(*layout.height_template, armor_id));
        }
    }

    auto evaluate_match(OutpostEKFParameters::Measurement const& measurement, Armor3d const& armor,
        double height_offset) const -> std::optional<double> {
        auto const H           = OutpostEKFParameters::H(ekf.x, armor, height_offset);
        auto const z_hat       = OutpostEKFParameters::h(ekf.x, armor, height_offset);
        auto const innovation  = OutpostEKFParameters::z_subtract(measurement.z, z_hat);
        auto const S           = H * ekf.P() * H.transpose() + measurement.R;
        auto const mahalanobis = rmcs::util::mahalanobis_distance(innovation, S);
        if (!mahalanobis.has_value() || *mahalanobis > mahalanobis_gate) return std::nullopt;

        return *mahalanobis;
    }

    auto reset_runtime_state(TimePoint t) -> void {
        color        = CampColor::UNKNOWN;
        ekf          = EKF {};
        layout       = OutpostArmorLayout {};
        time_stamp   = t;
        initialized  = false;
        update_count = 0;
    }

    auto select_best_match(std::span<Armor3d const> armors) const -> std::optional<MatchedArmor> {
        auto best_match       = std::optional<MatchedArmor> {};
        auto best_mahalanobis = std::numeric_limits<double>::infinity();

        for (auto const& armor : armors) {
            auto const measurement = OutpostEKFParameters::measurement(armor);

            visit_match_hypotheses([&](int armor_id, double height_offset,
                                       OutpostArmorLayout const& next_layout) {
                auto matched_armor = armor;
                matched_armor.id   = armor_id;

                auto const mahalanobis = evaluate_match(measurement, matched_armor, height_offset);
                if (!mahalanobis.has_value()) return;
                if (*mahalanobis >= best_mahalanobis) return;

                best_mahalanobis = *mahalanobis;
                best_match       = MatchedArmor { matched_armor, height_offset, next_layout };
            });
        }

        return best_match;
    }

    auto apply_match(MatchedArmor const& match) -> void {
        auto const measurement = OutpostEKFParameters::measurement(match.armor);

        ekf.update(
            measurement.z,
            [armor = match.armor, height_offset = match.height_offset](
                EKF::XVec const& x) { return OutpostEKFParameters::h(x, armor, height_offset); },
            [armor = match.armor, height_offset = match.height_offset](
                EKF::XVec const& x) { return OutpostEKFParameters::H(x, armor, height_offset); },
            measurement.R, OutpostEKFParameters::x_add, OutpostEKFParameters::z_subtract);

        layout = match.layout;
        update_count++;
    }

    CampColor color { CampColor::UNKNOWN };
    EKF ekf { EKF {} };
    OutpostArmorLayout layout {};
    TimePoint time_stamp;

    bool initialized { false };
    int update_count { 0 };
    std::chrono::duration<double> reset_interval { 1.5 };
    double mahalanobis_gate { 5.0 };
};

OutpostRobotState::OutpostRobotState() noexcept
    : OutpostRobotState(Clock::now()) { }

OutpostRobotState::OutpostRobotState(TimePoint stamp) noexcept
    : pimpl { std::make_unique<Impl>(stamp) } { }

OutpostRobotState::~OutpostRobotState() noexcept = default;

auto OutpostRobotState::initialize(Armor3d const& armor, TimePoint t) -> void {
    return pimpl->initialize(armor, t);
}

auto OutpostRobotState::predict(TimePoint t) -> void { return pimpl->predict(t); }

auto OutpostRobotState::update(std::span<Armor3d const> armors) -> bool {
    return pimpl->update(armors);
}

auto OutpostRobotState::is_converged() const -> bool { return pimpl->is_converged(); }

auto OutpostRobotState::get_snapshot() const -> Snapshot { return pimpl->get_snapshot(); }

auto OutpostRobotState::distance() const -> double { return pimpl->distance(); }

} // namespace rmcs::predictor

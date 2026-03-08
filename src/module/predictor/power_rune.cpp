#include "module/predictor/power_rune.hpp"

#include <cmath>
#include <numbers>
#include <optional>
#include <utility>
#include <variant>

#include "module/predictor/buff_ekf_parameter.hpp"
#include "module/predictor/clockwise_voter.hpp"
#include "module/predictor/power_rune_data_association.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/conversion.hpp"
#include "utility/time.hpp"

using namespace rmcs::predictor;

struct PowerRune::Impl {
    using EKFVariant       = std::variant<std::monostate, SmallEKF, LargeEKF>;
    using ClockwiseVoter   = PowerRuneClockwiseVoter;
    using ClockwiseTracker = PowerRuneClockwiseTracker;

    static constexpr auto kBladePhaseStep = PowerRuneDataAssociator::kBladePhaseStep;
    static constexpr auto kSpeedEpsilon   = 1e-6;

    template <typename TEKF>
    struct DataAssociationResult {
        typename TEKF::ZVec best_observation;
        int inferred_clockwise;
    };

    static auto observed_blade_angle(FanBlade3D const& blade) -> double {
        const auto orientation = blade.orientation.make<Eigen::Quaterniond>();
        const auto ypr         = util::eulers(orientation);
        return util::normalize_angle(ypr[2]);
    }
    template <typename TEKF>
    static auto make_observation(FanBlade3D const& blade)
        -> std::pair<typename TEKF::ZVec, double> {
        const auto blade_angle = observed_blade_angle(blade);

        auto r_mark_xyz = Eigen::Vector3d {};
        blade.R_mark_position.copy_to(r_mark_xyz);
        const auto r_mark_ypd = util::xyz2ypd(r_mark_xyz);

        auto blade_xyz = Eigen::Vector3d {};
        blade.position.copy_to(blade_xyz);
        const auto blade_ypd = util::xyz2ypd(blade_xyz);

        // z = [R_yaw, R_pitch, R_dist, blade_phase, B_yaw, B_pitch, B_dist]
        auto z = typename TEKF::ZVec {};
        z << r_mark_ypd(0), r_mark_ypd(1), r_mark_ypd(2), util::normalize_angle(blade_angle),
            blade_ypd(0), blade_ypd(1), blade_ypd(2);
        return { z, blade_angle };
    }

    static auto align_prior_blade_phase(double prior_blade_angle, double measured_blade_angle)
        -> double {
        // theta_aligned = argmin_{n in Z} |wrap(theta_obs - (theta_prior + n*2pi/5))|
        // 实现上在 n0±1 邻域搜索，n0 = round(wrap(theta_obs-theta_prior)/(2pi/5))。
        const auto prior = util::normalize_angle(prior_blade_angle);
        const auto obs   = util::normalize_angle(measured_blade_angle);
        const auto delta = util::normalize_angle(obs - prior);
        const auto n0    = static_cast<int>(std::lround(delta / kBladePhaseStep));

        auto best_angle = prior;
        auto best_error = std::abs(util::normalize_angle(obs - prior));
        for (int dn = -1; dn <= 1; ++dn) {
            const auto candidate =
                util::normalize_angle(prior + static_cast<double>(n0 + dn) * kBladePhaseStep);
            const auto error = std::abs(util::normalize_angle(obs - candidate));
            if (error < best_error) {
                best_error = error;
                best_angle = candidate;
            }
        }
        return best_angle;
    }

    template <typename TEKF>
    auto force_speed_sign(TEKF& kf) const -> void {
        if constexpr (std::is_same_v<std::decay_t<TEKF>, SmallEKF>) {
            // LargeEKF has no blade_speed state (x[6] is amplitude), so only SmallEKF applies.
            // blade_w <- sgn(clockwise) * |blade_w|, with epsilon dead-zone.
            auto& speed = kf.x(BuffEKFParameters::kIdxSmallBladeSpeed);
            if (std::abs(speed) < kSpeedEpsilon) {
                speed = 0.0;
                return;
            }
            speed = static_cast<double>(clockwise_) * std::abs(speed);
        }
    }

    template <typename TEKF>
    auto data_association(TEKF const& kf, FanBlade3D const& blade)
        -> std::optional<DataAssociationResult<TEKF>> {
        const auto [raw_observation, observed_blade_angle] = make_observation<TEKF>(blade);
        const auto inferred_clockwise = clockwise_tracker_.update(observed_blade_angle, time_stamp);

        const auto predicted_observation    = BuffEKFParameters::rune_h<TEKF>(kf.x);
        const auto observation_jacobian     = BuffEKFParameters::rune_H<TEKF>(kf.x);
        static const auto observation_noise = BuffEKFParameters::rune_R<TEKF>();
        const auto association = PowerRuneDataAssociator::associate<TEKF>(raw_observation,
            observed_blade_angle, predicted_observation, observation_jacobian, kf.P(),
            observation_noise, BuffEKFParameters::rune_z_subtract<TEKF>);
        if (!association.has_value()) {
            return std::nullopt;
        }

        return DataAssociationResult<TEKF> {
            .best_observation   = association->best_z,
            .inferred_clockwise = inferred_clockwise,
        };
    }

    template <typename TEKF>
    auto state_correction(TEKF& kf, typename TEKF::ZVec const& best_observation) const -> void {
        // Multi-blade symmetry may jump phase by ±2π/5; align prior to nearest sector.
        kf.x(BuffEKFParameters::kIdxBladePhase) =
            align_prior_blade_phase(kf.x(BuffEKFParameters::kIdxBladePhase), best_observation(3));
        // Enforce direction sign consistency before correction.
        force_speed_sign(kf);

        // Update step:
        // y = z ⊖ h(x),  K = P H^T (H P H^T + R)^-1
        // x = x ⊕ K y
        kf.update(best_observation, BuffEKFParameters::rune_h<TEKF>,
            BuffEKFParameters::rune_H<TEKF>, BuffEKFParameters::rune_R<TEKF>(),
            BuffEKFParameters::rune_x_add<TEKF>, BuffEKFParameters::rune_z_subtract<TEKF>);

        // Re-apply after update because EKF correction can flip sign numerically.
        force_speed_sign(kf);
    }

    explicit Impl()
        : mode_ { std::nullopt }
        , ekf_ { std::monostate {} }
        , clockwise_ { 1 }
        , clockwise_tracker_ {}
        , time_stamp { Clock::now() }
        , last_update_stamp { std::nullopt }
        , initialized { false } { }

    auto initialize(FanBlade3D const& fan_blade, PowerRuneMode mode, int clockwise,
        Clock::time_point t) -> void {
        mode_                  = mode;
        const auto blade_angle = observed_blade_angle(fan_blade);
        clockwise_             = clockwise_tracker_.reset(clockwise, blade_angle, t);

        if (mode_ == PowerRuneMode::SMALL) {
            ekf_ = SmallEKF { BuffEKFParameters::rune_x<SmallEKF>(fan_blade, clockwise_),
                BuffEKFParameters::rune_P<SmallEKF>() };
        } else {
            ekf_ = LargeEKF { BuffEKFParameters::rune_x<LargeEKF>(fan_blade, clockwise_),
                BuffEKFParameters::rune_P<LargeEKF>() };
        }

        std::visit(
            [this](auto& kf) {
                using T = std::decay_t<decltype(kf)>;
                if constexpr (!std::is_same_v<T, std::monostate>) {
                    kf.x(BuffEKFParameters::kIdxBladePhase) =
                        util::normalize_angle(kf.x(BuffEKFParameters::kIdxBladePhase));
                    kf.x(BuffEKFParameters::kIdxRuneYaw) =
                        util::normalize_angle(kf.x(BuffEKFParameters::kIdxRuneYaw));
                    force_speed_sign(kf);
                }
            },
            ekf_);

        initialized       = true;
        time_stamp        = t;
        last_update_stamp = t;
    }

    auto predict(Clock::time_point t) -> void {
        if (!initialized) {
            time_stamp        = t;
            last_update_stamp = t;
            return;
        }

        const auto dt = util::delta_time(t, time_stamp);
        if (dt < std::chrono::duration<double>::zero() || dt > reset_interval) {
            reset();
            time_stamp        = t;
            last_update_stamp = t;
            return;
        }
        const auto dt_s = dt.count();

        std::visit(
            [this, dt_s](auto& kf) {
                using T = std::decay_t<decltype(kf)>;
                if constexpr (!std::is_same_v<T, std::monostate>) {
                    // Predict step:
                    // x_{k|k-1} = f(x_{k-1|k-1}, dt, clockwise)
                    // P_{k|k-1} = F P F^T + Q
                    kf.predict(
                        BuffEKFParameters::rune_f<T>(dt_s, clockwise_),
                        [this, dt_s](typename T::XVec const& x) {
                            return BuffEKFParameters::rune_F<T>(dt_s, x, clockwise_);
                        },
                        BuffEKFParameters::rune_Q<T>(dt_s));
                    force_speed_sign(kf);
                }
            },
            ekf_);

        time_stamp = t;
    }

    auto update(FanBlade3D const& blade) -> void {
        if (!initialized) {
            time_stamp = last_update_stamp.value_or(Clock::now());
            if (!mode_.has_value()) {
                last_update_stamp = time_stamp;
                return;
            }
            initialize(blade, *mode_, clockwise_, time_stamp);
            return;
        }

        std::visit(
            [this, &blade](auto& kf) {
                using T = std::decay_t<decltype(kf)>;
                if constexpr (!std::is_same_v<T, std::monostate>) {
                    const auto association_result = data_association<T>(kf, blade);
                    if (!association_result.has_value()) {
                        return;
                    }
                    clockwise_ = association_result->inferred_clockwise;
                    state_correction<T>(kf, association_result->best_observation);
                }
            },
            ekf_);
        last_update_stamp = time_stamp;
    }

    auto get_snapshot() const -> Snapshot {
        auto x = std::visit(
            [](auto const& kf) -> Snapshot::XVariant {
                using T = std::decay_t<decltype(kf)>;
                if constexpr (std::is_same_v<T, std::monostate>) {
                    return std::monostate {};
                } else {
                    return kf.x;
                }
            },
            ekf_);
        return Snapshot { std::move(x), mode_, clockwise_, time_stamp };
    }

    auto reset() -> void {
        initialized = false;
        ekf_        = std::monostate {};
        clockwise_  = clockwise_tracker_.reset(clockwise_);
        last_update_stamp.reset();
    }

    PowerRuneModeOpt mode_;
    EKFVariant ekf_;
    int clockwise_;
    ClockwiseTracker clockwise_tracker_;
    Clock::time_point time_stamp;
    std::optional<Clock::time_point> last_update_stamp;
    bool initialized;
    const std::chrono::duration<double> reset_interval { 1.0 };
};

PowerRune::PowerRune() noexcept
    : pimpl { std::make_unique<Impl>() } { }
PowerRune::~PowerRune() noexcept = default;

auto PowerRune::initialize(
    FanBlade3D const& fan_blade, PowerRuneMode mode, int clockwise, Clock::time_point t) -> void {
    pimpl->initialize(fan_blade, mode, clockwise, t);
}
auto PowerRune::predict(Clock::time_point t) -> void { pimpl->predict(t); }
auto PowerRune::update(FanBlade3D const& blade) -> void { pimpl->update(blade); }
auto PowerRune::reset() -> void { pimpl->reset(); }
auto PowerRune::get_snapshot() const -> Snapshot { return pimpl->get_snapshot(); }

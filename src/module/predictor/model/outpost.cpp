#include "outpost.hpp"

#include "utility/math/angle.hpp"
#include "utility/robot/constant.hpp"

#include <eigen3/Eigen/Geometry>

using namespace rmcs;
using namespace rmcs::util;

struct OutpostModel::Impl {
    using StateVector         = Eigen::Matrix<double, 5, 1>;
    using Covariance          = Eigen::Matrix<double, 5, 5>;
    using ProcessNoise        = Eigen::Matrix<double, 5, 5>;
    using ObservationNoise    = Eigen::Matrix<double, 4, 4>;
    using ObservationVector   = Eigen::Matrix<double, 4, 1>;
    using ObservationJacobian = Eigen::Matrix<double, 4, 5>;
    using KalmanGain          = Eigen::Matrix<double, 5, 4>;

    struct Context {
        StateVector posteriors_state       = StateVector::Zero();
        Covariance posteriors_covariance   = Covariance::Identity();
        ProcessNoise noise_process         = ProcessNoise::Zero();
        ObservationNoise noise_observation = ObservationNoise::Zero();

        Context() noexcept {
            noise_process.diagonal() << 1e-6, 1e-6, 1e-6, 1e-3, 1e-3;
            noise_observation.diagonal() << 0.09, 0.09, 0.09, 5e-3;
        }

        auto get_state() const noexcept {
            return State {
                .x = posteriors_state[0],
                .y = posteriors_state[1],
                .z = posteriors_state[2],

                .rotation_speed = posteriors_state[3],
                .rotation_angle = posteriors_state[4],
            };
        }

        auto update_center(const Eigen::Vector3d& point) noexcept {
            posteriors_state[0] = point.x();
            posteriors_state[1] = point.y();
            posteriors_state[2] = point.z();
        }
    };

    static constexpr auto kOffsetTable = std::array {
        std::tuple { 0 * std::numbers::pi * 2 / 3, -0 * kOutpostArmorHeightStep }, // 高
        std::tuple { 1 * std::numbers::pi * 2 / 3, -1 * kOutpostArmorHeightStep }, // 中
        std::tuple { 2 * std::numbers::pi * 2 / 3, -2 * kOutpostArmorHeightStep }, // 低
    };
    std::uint8_t ref_index = 0; // 参考板的序号
    std::uint8_t obs_index = 0; // 当前板的序号

    Context context;

    static auto make_observation_jacobian() {
        auto jacobian = ObservationJacobian { };
        // clang-format off
        jacobian <<
            1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 0, 1;
        // clang-format on
        return jacobian;
    }

    auto initialize_from(const Armor3d& armor) noexcept -> void {
        constexpr auto pitch  = kPredictedOutpostArmorPitch;
        constexpr auto radius = kOutpostRadius;

        const auto translation = armor.translation.make<Eigen::Vector3d>();
        const auto orientation = armor.orientation.make<Eigen::Quaterniond>();

        const auto backward        = Eigen::Vector3d { orientation * Eigen::Vector3d::UnitX() };
        const auto armor_to_center = Eigen::Vector3d {
            Eigen::AngleAxisd { -pitch, orientation * Eigen::Vector3d::UnitY() } * backward
        };
        const auto center = Eigen::Vector3d { translation + radius * armor_to_center };

        context.update_center(center);
        context.posteriors_state[3] = 0.0;
        context.posteriors_state[4] =
            util::normalize_angle(std::atan2(-armor_to_center.y(), -armor_to_center.x()));

        auto initial_covariance_diagonal = Eigen::Matrix<double, 5, 1> { };
        initial_covariance_diagonal << 1.0, 1.0, 1.0, 64.0, 0.4;

        context.posteriors_covariance = initial_covariance_diagonal.asDiagonal();
    }

    static auto predict_state(double dt, const StateVector& last) -> StateVector {
        auto next = last;

        next[4] = util::normalize_angle(last[4] + last[3] * dt);
        return next;
    }

    auto predict_covariance(double dt, const Covariance& covariance) const -> Covariance {
        auto jacobian  = Covariance { Covariance::Identity() };
        jacobian(4, 3) = dt;
        return jacobian * covariance * jacobian.transpose() + context.noise_process;
    }

    static auto measure_innovation(
        const ObservationVector& observation, const StateVector& prior_state) -> ObservationVector {

        auto predicted_observation = ObservationVector { };
        predicted_observation << prior_state[0], prior_state[1], prior_state[2], prior_state[4];

        auto innovation        = ObservationVector { observation - predicted_observation };
        innovation.coeffRef(3) = util::normalize_angle(innovation.coeff(3));
        return innovation;
    }

    auto calculate_kalman_gain(const Covariance& prior_covariance) const -> KalmanGain {
        const auto jacobian = make_observation_jacobian();

        const auto innovation_covariance =
            jacobian * prior_covariance * jacobian.transpose() + context.noise_observation;

        return prior_covariance * jacobian.transpose()
            * innovation_covariance.ldlt().solve(ObservationNoise::Identity());
    }

    auto a_posteriori_update(const StateVector& prior_state, const Covariance& prior_covariance,
        const KalmanGain& kalman_gain, const ObservationVector& innovation,
        const ObservationJacobian& jacobian) const -> std::pair<StateVector, Covariance> {

        auto posterior_state = StateVector { prior_state };
        posterior_state.noalias() += kalman_gain * innovation;
        posterior_state[4] = util::normalize_angle(posterior_state[4]);

        const auto complement          = Covariance::Identity() - kalman_gain * jacobian;
        auto posterior_covariance      = Covariance { };
        posterior_covariance.noalias() = complement * prior_covariance * complement.transpose();
        posterior_covariance +=
            (kalman_gain * context.noise_observation * kalman_gain.transpose()).eval();
        posterior_covariance = 0.5 * (posterior_covariance + posterior_covariance.transpose());

        return { posterior_state, posterior_covariance };
    }

    auto predict(double dt) noexcept -> void {
        const auto prior_state      = predict_state(dt, context.posteriors_state);
        const auto prior_covariance = predict_covariance(dt, context.posteriors_covariance);

        context.posteriors_state      = prior_state;
        context.posteriors_covariance = prior_covariance;
    }

    auto correct(const Armor3d& armor) noexcept -> void {
        { // 切板检测
        }

        constexpr auto pitch = kPredictedOutpostArmorPitch;

        const auto orientation = armor.orientation.make<Eigen::Quaterniond>();
        const auto translation = armor.translation.make<Eigen::Vector3d>();

        const auto backward        = Eigen::Vector3d { orientation * Eigen::Vector3d::UnitX() };
        const auto armor_to_center = Eigen::Vector3d {
            Eigen::AngleAxisd { -pitch, orientation * Eigen::Vector3d::UnitY() } * backward
        };

        const auto center = Eigen::Vector3d { translation + kOutpostRadius * armor_to_center };

        const auto top_armor_yaw =
            util::normalize_angle(std::atan2(armor_to_center.y(), armor_to_center.x()));

        auto observation = ObservationVector { };
        observation << center.x(), center.y(), center.z(), top_armor_yaw;

        const auto prior_state      = context.posteriors_state;
        const auto prior_covariance = context.posteriors_covariance;

        const auto innovation  = measure_innovation(observation, prior_state);
        const auto kalman_gain = calculate_kalman_gain(prior_covariance);
        const auto jacobian    = make_observation_jacobian();

        const auto [posterior_state, posterior_covariance] =
            a_posteriori_update(prior_state, prior_covariance, kalman_gain, innovation, jacobian);

        context.posteriors_state      = posterior_state;
        context.posteriors_covariance = posterior_covariance;
    }
};

OutpostModel::OutpostModel(const Armor3d& armor) noexcept
    : pimpl { std::make_unique<Impl>() } {
    pimpl->initialize_from(armor);
}

OutpostModel::~OutpostModel() noexcept = default;

auto OutpostModel::state() noexcept -> State { return pimpl->context.get_state(); }

auto OutpostModel::predict(double dt) noexcept -> void { pimpl->predict(dt); }

auto OutpostModel::correct(const Armor3d& armor) noexcept -> void { pimpl->correct(armor); }

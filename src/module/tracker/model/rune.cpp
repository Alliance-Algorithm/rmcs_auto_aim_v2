#include "rune.hpp"
#include "rune_energy_fitter.hpp"

#include "utility/clock.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/camera.hpp"
#include "utility/math/conversion.hpp"
#include "utility/math/hungarian.hpp"
#include "utility/math/mahalanobis.hpp"
#include "utility/math/reprojection.hpp"
#include "utility/math/solve_pnp/pnp_solution.hpp"
#include "utility/robot/constant.hpp"

#include <eigen3/Eigen/Geometry>

#include <array>
#include <chrono>
#include <cstdio>
#include <limits>
#include <numbers>
#include <optional>
#include <ranges>

using namespace rmcs;
using namespace rmcs::util;

// ===== State Methods =====

auto RuneModel::State::transition(double seconds) -> void {
    if (sine_valid) {
        sine_t += seconds;
        sine_phase += sine_omega * seconds;
        rotation_angle = sine_C + sine_v * sine_t - sine_a / sine_omega * std::cos(sine_phase);
        rotation_speed = sine_v + sine_a * std::sin(sine_phase);
    } else {
        rotation_angle = rotation_angle + rotation_speed * seconds;
    }
}

auto RuneModel::State::get_direction() const -> Point3d { return Point3d { x, y, z }; }

auto RuneModel::State::get_rotation_speed() const -> double { return rotation_speed; }

auto RuneModel::State::get_aimpoints() const -> std::vector<Point3d> {
    const auto r_face = Eigen::AngleAxisd { face_yaw, Eigen::Vector3d::UnitZ() };

    static constexpr std::array kBladeAnglesDeg = { 0.0, 72.0, 144.0, 216.0, 288.0 };

    auto result = std::vector<Point3d> { };
    for (const auto& [deg, inactive] : std::views::zip(kBladeAnglesDeg, inactive)) {
        if (!inactive) continue;

        double alpha   = rotation_angle + util::deg2rad(deg);
        double local_y = -kRuneGlobalRadius * std::sin(alpha);
        double local_z = kRuneGlobalRadius * std::cos(alpha);
        Eigen::Vector3d world =
            Eigen::Vector3d(x, y, z) + r_face * Eigen::Vector3d(0, local_y, local_z);

        result.emplace_back(world.x(), world.y(), world.z());
        break;
    }
    return result;
}

// ===== Impl =====

struct RuneModel::Impl {
    using StateVector      = Eigen::Matrix<double, 6, 1>;
    using Covariance       = Eigen::Matrix<double, 6, 6>;
    using ProcessNoise     = Eigen::Matrix<double, 6, 6>;
    using ObservationNoise = Eigen::Matrix<double, 2, 2>;
    using KalmanGain       = Eigen::Matrix<double, 6, 2>;
    using ObsJacobian      = Eigen::Matrix<double, 2, 6>;

    static constexpr auto kX   = 0;
    static constexpr auto kY   = 1;
    static constexpr auto kZ   = 2;
    static constexpr auto kW   = 3;
    static constexpr auto kA   = 4;
    static constexpr auto kPsi = 5;

    struct Context {
        StateVector posteriors_state     = StateVector::Zero();
        Covariance posteriors_covariance = Covariance::Identity();
        ProcessNoise noise_process       = ProcessNoise::Zero();

        double prediction_speed   = 0.0;
        bool use_prediction_speed = false;
        double prediction_cost    = 0.0;

        double sine_C     = 0.0;
        double sine_v     = 0.0;
        double sine_a     = 0.0;
        double sine_omega = 0.0;
        double sine_phase = 0.0;
        double sine_t     = 0.0;
        bool sine_valid   = false;
        double sine_cost  = 0.0;

        auto set_noise(const Config& config) noexcept {
            noise_process.diagonal() << config.noise_x, config.noise_y, config.noise_z,
                config.noise_rotation_speed, config.noise_rotation_angle, config.noise_face_yaw;
        }

        auto reset_covariance() noexcept {
            auto diag = StateVector { };
            diag << 64.0, 64.0, 64.0, 100.0, 10.0, 64.0;
            posteriors_covariance = diag.asDiagonal();
        }

        auto get_state(const std::array<bool, 5>& inactive) const noexcept {
            return State {
                .x              = posteriors_state[kX],
                .y              = posteriors_state[kY],
                .z              = posteriors_state[kZ],
                .rotation_speed = use_prediction_speed ? prediction_speed : posteriors_state[kW],
                .rotation_angle = posteriors_state[kA],
                .face_yaw       = posteriors_state[kPsi],
                .inactive       = inactive,
                .use_prediction_speed = use_prediction_speed,
                .prediction_cost      = sine_valid ? sine_cost : prediction_cost,
                .sine_C               = sine_C,
                .sine_v               = sine_v,
                .sine_a               = sine_a,
                .sine_omega           = sine_omega,
                .sine_phase           = sine_phase,
                .sine_t               = sine_t,
                .sine_valid           = sine_valid,
            };
        }
    };

    struct Observable {
        CameraFeature feature;

        static constexpr int kFeatureIcon  = 0;
        static constexpr int kFeatureCount = 6;

        std::array<Eigen::Vector2d, kFeatureCount> projected;
        std::array<Eigen::Vector3d, kFeatureCount> in_camera;
        std::array<bool, kFeatureCount> visible;

        auto update(const StateVector& state) noexcept {
            const auto theta  = state[kA];
            const auto psi    = state[kPsi];
            const auto center = Eigen::Vector3d(state[kX], state[kY], state[kZ]);
            const auto r_face =
                Eigen::AngleAxisd { psi, Eigen::Vector3d::UnitZ() }.toRotationMatrix();

            const auto q_odom_from_cam = feature.orientation.make<Eigen::Quaterniond>();
            const auto q_cam_from_odom = q_odom_from_cam.conjugate();
            const auto cam_position    = feature.translation.make<Eigen::Vector3d>();

            static constexpr std::array kBladeDeg = { 0.0, 72.0, 144.0, 216.0, 288.0 };

            for (int i = 0; i < kFeatureCount; ++i) {
                Eigen::Vector3d local;
                if (i == kFeatureIcon) {
                    local = Eigen::Vector3d(-kRuneIconProminentDistance, 0, 0);
                } else {
                    const auto alpha = theta + util::deg2rad(kBladeDeg[i - 1]);
                    local << 0, -kRuneGlobalRadius * std::sin(alpha),
                        kRuneGlobalRadius * std::cos(alpha);
                }

                Eigen::Vector3d world  = center + r_face * local;
                Eigen::Vector3d camera = q_cam_from_odom * (world - cam_position);

                in_camera[i] = camera;

                constexpr auto kMinZ     = 0.1;
                const auto point_ocv     = util::ros2opencv_position(camera);
                const auto projected_opt = util::reproject_point(Point3d { point_ocv }, feature);
                if (point_ocv.z() > kMinZ && projected_opt) {
                    projected[i] = Eigen::Vector2d { projected_opt->x, projected_opt->y };
                    visible[i]   = true;
                } else {
                    projected[i].setZero();
                    visible[i] = false;
                }
            }
        }
    };

    Config config { };
    CameraFeature camera_feature;
    Observable observable;

    Context context;
    std::array<bool, 5> blade_inactive { };
    Addition addition { };
    Timestamp init_timestamp = Clock::now();
    Timestamp current_stamp;
    std::size_t update_count = 0;
    RuneEnergyFitter fitter;

    explicit Impl(const Config& cfg) noexcept {
        config = cfg;
        context.set_noise(config);
        context.reset_covariance();
    }

    auto configure_camera(std::array<double, 9> matrix, std::array<double, 5> coeff) noexcept {
        camera_feature.from(matrix);
        camera_feature.from(coeff);
        observable.feature.from(matrix);
        observable.feature.from(coeff);
    }

    auto update_transform(const Transform& t) noexcept -> void {
        camera_feature.translation     = t.translation;
        camera_feature.orientation     = t.orientation;
        observable.feature.translation = t.translation;
        observable.feature.orientation = t.orientation;
    }

    struct InitCandidate {
        Eigen::Vector3d center_odom { Eigen::Vector3d::Zero() };
        double face_yaw_odom = 0.0;

        int inactive_inliers = 0;
        double icon_error    = std::numeric_limits<double>::max();

        double seed_sse       = std::numeric_limits<double>::max();
        double seed_max_error = std::numeric_limits<double>::max();

        double inactive_center_sse = std::numeric_limits<double>::max();
    };

    static auto make_seed_image_points(const RuneIcon& icon, const RuneBullseye& bullseye)
        -> std::optional<std::array<cv::Point2f, 5>> {
        constexpr auto kEps = 1e-6;

        const auto icon_point   = icon.center.make<cv::Point2f>();
        const auto center_point = bullseye.center.make<cv::Point2f>();

        auto corners = std::array<cv::Point2f, 4> { };
        std::ranges::copy(bullseye.corners | std::views::transform([](const Point2d& point) {
            return point.make<cv::Point2f>();
        }),
            corners.begin());

        auto index_b      = std::size_t { 0 };
        auto index_t      = std::size_t { 0 };
        auto min_distance = std::numeric_limits<double>::max();
        auto max_distance = -std::numeric_limits<double>::max();
        for (const auto& [index, corner] : corners | std::views::enumerate) {
            const auto dx       = static_cast<double>(corner.x - icon_point.x);
            const auto dy       = static_cast<double>(corner.y - icon_point.y);
            const auto distance = std::hypot(dx, dy);

            if (distance < min_distance) {
                index_b      = static_cast<std::size_t>(index);
                min_distance = distance;
            }
            if (distance > max_distance) {
                index_t      = static_cast<std::size_t>(index);
                max_distance = distance;
            }
        }
        if (index_b == index_t) return std::nullopt;

        auto remaining       = std::array<std::size_t, 2> { };
        auto remaining_count = std::size_t { 0 };
        for (std::size_t index = 0; index < corners.size(); ++index) {
            if (index == index_b || index == index_t) continue;
            if (remaining_count >= remaining.size()) return std::nullopt;

            remaining[remaining_count] = index;
            remaining_count++;
        }
        if (remaining_count != remaining.size()) return std::nullopt;

        const auto b = corners[index_b];
        const auto t = corners[index_t];

        auto l = cv::Point2f { };
        auto r = cv::Point2f { };
        {
            const auto z_axis_raw = t - b;
            const auto z_axis_len =
                std::hypot(static_cast<double>(z_axis_raw.x), static_cast<double>(z_axis_raw.y));
            if (z_axis_len <= kEps) return std::nullopt;

            const auto z_axis = cv::Point2d {
                static_cast<double>(z_axis_raw.x) / z_axis_len,
                static_cast<double>(z_axis_raw.y) / z_axis_len,
            };
            const auto y_axis = cv::Point2d { z_axis.y, -z_axis.x };

            const auto first  = corners[remaining[0]];
            const auto second = corners[remaining[1]];

            const auto first_projection = static_cast<double>(first.x - center_point.x) * y_axis.x
                + static_cast<double>(first.y - center_point.y) * y_axis.y;
            const auto second_projection = static_cast<double>(second.x - center_point.x) * y_axis.x
                + static_cast<double>(second.y - center_point.y) * y_axis.y;
            if (std::abs(first_projection - second_projection) <= kEps) return std::nullopt;

            if (first_projection > second_projection) {
                l = first;
                r = second;
            } else {
                l = second;
                r = first;
            }
        }

        return std::array<cv::Point2f, 5> { icon_point, t, l, b, r };
    }

    auto evaluate_seed_reprojection(const RuneIcon& icon, const RuneBullseye& bullseye,
        const Eigen::Vector3d& t_camera_rune, const Eigen::Quaterniond& q_camera_rune) const
        -> std::optional<std::pair<double, double>> {

        const auto image_points_opt = make_seed_image_points(icon, bullseye);
        if (!image_points_opt) return std::nullopt;

        const auto rotation_opencv    = util::ros2opencv_rotation(q_camera_rune.toRotationMatrix());
        const auto translation_opencv = util::ros2opencv_position(t_camera_rune);

        auto camera_points = std::array<cv::Point3f, 5> { };
        for (const auto& [index, point] : RunePagePoints::kPoints | std::views::enumerate) {
            const auto p_local  = point.make<Eigen::Vector3d>();
            const auto p_camera = rotation_opencv * p_local + translation_opencv;
            camera_points[static_cast<std::size_t>(index)] = cv::Point3f {
                static_cast<float>(p_camera.x()),
                static_cast<float>(p_camera.y()),
                static_cast<float>(p_camera.z()),
            };
        }

        auto solution                = ReprojectionSolution<5> { };
        solution.input.camera        = camera_feature;
        solution.input.object_points = camera_points;
        solution.input.image_points  = *image_points_opt;
        if (!solution.solve()) return std::nullopt;

        auto sse       = 0.0;
        auto max_error = 0.0;
        for (const auto& [projected, detected] :
            std::views::zip(solution.result.projected_points, solution.input.image_points)) {
            const auto dx   = static_cast<double>(projected.x - detected.x);
            const auto dy   = static_cast<double>(projected.y - detected.y);
            const auto err2 = dx * dx + dy * dy;
            sse += err2;
            max_error = std::max(max_error, std::sqrt(err2));
        }
        return std::pair { sse, max_error };
    }

    auto evaluate_inactive_centers(std::span<const RuneBullseye> inactive_bullseyes,
        const Eigen::Vector3d& center_odom, double face_yaw_odom) const -> std::pair<int, double> {

        auto simulated    = observable;
        StateVector state = StateVector::Zero();
        state[kX]         = center_odom.x();
        state[kY]         = center_odom.y();
        state[kZ]         = center_odom.z();
        state[kW]         = 0.0;
        state[kA]         = 0.0;
        state[kPsi]       = face_yaw_odom;
        simulated.update(state);

        constexpr auto kBladeBegin = 1;
        constexpr auto kBladeEnd   = Observable::kFeatureCount;
        const auto gate2           = config.init_center_gate * config.init_center_gate;

        auto obs_points = std::array<Eigen::Vector2d, 2> { };
        for (const auto& [index, bullseye] : inactive_bullseyes | std::views::enumerate) {
            obs_points[static_cast<std::size_t>(index)] =
                Eigen::Vector2d { bullseye.center.x, bullseye.center.y };
        }

        auto single_best = std::array<double, 2> {
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
        };
        for (const auto& [obs_index, obs_point] : obs_points | std::views::enumerate) {
            for (int blade = kBladeBegin; blade < kBladeEnd; ++blade) {
                if (!simulated.visible[blade]) continue;
                const auto diff = obs_point - simulated.projected[blade];
                const auto err2 = diff.squaredNorm();
                if (err2 < single_best[static_cast<std::size_t>(obs_index)]) {
                    single_best[static_cast<std::size_t>(obs_index)] = err2;
                }
            }
        }

        if (inactive_bullseyes.size() == 1) {
            if (single_best[0] <= gate2) return { 1, single_best[0] };
            return { 0, std::numeric_limits<double>::max() };
        }

        auto partial_inliers = 0;
        auto partial_sse     = 0.0;
        for (double err2 : single_best) {
            if (err2 <= gate2) {
                partial_inliers += 1;
                partial_sse += err2;
            }
        }

        auto best_pair = std::numeric_limits<double>::max();
        for (int lhs = kBladeBegin; lhs < kBladeEnd; ++lhs) {
            if (!simulated.visible[lhs]) continue;
            for (int rhs = kBladeBegin; rhs < kBladeEnd; ++rhs) {
                if (lhs == rhs || !simulated.visible[rhs]) continue;

                const auto lhs_err2 = (obs_points[0] - simulated.projected[lhs]).squaredNorm();
                const auto rhs_err2 = (obs_points[1] - simulated.projected[rhs]).squaredNorm();
                if (lhs_err2 > gate2 || rhs_err2 > gate2) continue;

                best_pair = std::min(best_pair, lhs_err2 + rhs_err2);
            }
        }
        if (best_pair != std::numeric_limits<double>::max()) return { 2, best_pair };
        if (partial_inliers > 0) return { 1, std::min(single_best[0], single_best[1]) };
        return { 0, std::numeric_limits<double>::max() };
    }

    auto evaluate_reduced_layout(const RuneIcon& icon,
        std::span<const RuneBullseye> inactive_bullseyes, const Eigen::Vector3d& center_odom,
        double face_yaw_odom) const -> std::optional<std::pair<double, std::pair<int, double>>> {

        auto simulated    = observable;
        StateVector state = StateVector::Zero();
        state[kX]         = center_odom.x();
        state[kY]         = center_odom.y();
        state[kZ]         = center_odom.z();
        state[kPsi]       = face_yaw_odom;
        simulated.update(state);

        if (!simulated.visible[Observable::kFeatureIcon]) return std::nullopt;

        const auto icon_observation = Eigen::Vector2d { icon.center.x, icon.center.y };
        const auto icon_error =
            (icon_observation - simulated.projected[Observable::kFeatureIcon]).squaredNorm();

        const auto inactive_score =
            evaluate_inactive_centers(inactive_bullseyes, center_odom, face_yaw_odom);
        return std::pair { icon_error, inactive_score };
    }

    auto make_init_candidate(const RuneIcon& icon, const RuneBullseye& seed,
        std::span<const RuneBullseye> inactive_bullseyes) const -> std::optional<InitCandidate> {

        auto pnp          = SingleRunePnpSolution { };
        pnp.input.cam     = camera_feature;
        pnp.input.center  = seed.center;
        pnp.input.icon    = icon.center;
        pnp.input.corners = seed.corners;
        if (!pnp.solve()) return std::nullopt;

        const auto t_camera_rune = pnp.result.translation.make<Eigen::Vector3d>();
        const auto q_camera_rune = pnp.result.orientation.make<Eigen::Quaterniond>().normalized();

        const auto seed_error =
            evaluate_seed_reprojection(icon, seed, t_camera_rune, q_camera_rune);
        if (!seed_error) return std::nullopt;

        const auto seed_mean_error = std::sqrt(seed_error->first / 5.0);
        if (seed_mean_error > config.init_seed_mean_error) return std::nullopt;
        if (seed_error->second > config.init_seed_max_error) return std::nullopt;

        const auto q_odom_camera = camera_feature.orientation.make<Eigen::Quaterniond>();
        const auto t_odom_camera = camera_feature.translation.make<Eigen::Vector3d>();

        auto candidate           = InitCandidate { };
        const auto q_face_odom   = (q_odom_camera * q_camera_rune).normalized();
        const auto face_x_odom   = q_face_odom * Eigen::Vector3d::UnitX();
        const auto base_yaw      = std::atan2(face_x_odom.y(), face_x_odom.x());
        candidate.center_odom    = q_odom_camera * t_camera_rune + t_odom_camera;
        candidate.seed_sse       = seed_error->first;
        candidate.seed_max_error = seed_error->second;

        static constexpr std::array kYawOffsets = {
            0.0,
            std::numbers::pi / 2.0,
            -std::numbers::pi / 2.0,
            std::numbers::pi,
        };

        auto best_yaw_score = std::optional<std::tuple<double, double, int, double>> { };
        for (double offset : kYawOffsets) {
            const auto yaw = util::normalize_angle(base_yaw + offset);

            const auto face_x_candidate_odom =
                Eigen::Vector3d { std::cos(yaw), std::sin(yaw), 0.0 };
            const auto face_x_candidate_camera = q_odom_camera.conjugate() * face_x_candidate_odom;
            if (face_x_candidate_camera.x() <= 0.0) continue;

            const auto layout_score =
                evaluate_reduced_layout(icon, inactive_bullseyes, candidate.center_odom, yaw);
            if (!layout_score) continue;

            const auto& [icon_error, inactive_score]           = *layout_score;
            const auto [inactive_inliers, inactive_center_sse] = inactive_score;

            const auto current = std::tuple {
                icon_error,
                yaw,
                inactive_inliers,
                inactive_center_sse,
            };
            if (!best_yaw_score) {
                best_yaw_score = current;
                continue;
            }

            const auto& [best_icon_error, best_yaw, best_inliers, best_center_sse] =
                *best_yaw_score;
            std::ignore = best_yaw;
            if (inactive_inliers > best_inliers
                || (inactive_inliers == best_inliers
                    && (icon_error < best_icon_error
                        || (icon_error == best_icon_error
                            && inactive_center_sse < best_center_sse)))) {
                best_yaw_score = current;
            }
        }

        if (!best_yaw_score) return std::nullopt;

        candidate.icon_error          = std::get<0>(*best_yaw_score);
        candidate.face_yaw_odom       = std::get<1>(*best_yaw_score);
        candidate.inactive_inliers    = std::get<2>(*best_yaw_score);
        candidate.inactive_center_sse = std::get<3>(*best_yaw_score);
        return candidate;
    }

    static auto better_init_candidate(const InitCandidate& lhs, const InitCandidate& rhs) {
        if (lhs.inactive_inliers != rhs.inactive_inliers) {
            return lhs.inactive_inliers > rhs.inactive_inliers;
        }
        if (lhs.icon_error != rhs.icon_error) {
            return lhs.icon_error < rhs.icon_error;
        }
        if (lhs.seed_max_error != rhs.seed_max_error) {
            return lhs.seed_max_error < rhs.seed_max_error;
        }
        if (lhs.seed_sse != rhs.seed_sse) {
            return lhs.seed_sse < rhs.seed_sse;
        }
        return lhs.inactive_center_sse < rhs.inactive_center_sse;
    }

    // ===== Prediction =====

    static auto predict_state(double dt, const StateVector& last) -> StateVector {
        auto next  = last;
        next[kA]   = last[kA] + last[kW] * dt;
        next[kPsi] = util::normalize_angle(last[kPsi]);
        return next;
    }

    auto predict_covariance(double dt, const Covariance& covariance) const -> Covariance {
        auto jacobian    = Covariance { Covariance::Identity() };
        jacobian(kA, kW) = dt;
        return jacobian * covariance * jacobian.transpose() + context.noise_process;
    }

    // ===== Observation Jacobian =====

    auto make_observation_jacobian(int feature_id) const -> ObsJacobian {
        const auto& state = context.posteriors_state;
        if (state.hasNaN()) return ObsJacobian::Zero();

        const auto theta = state[kA];
        const auto psi   = state[kPsi];
        const auto r_cam_from_odom =
            camera_feature.orientation.make<Eigen::Quaterniond>().conjugate().toRotationMatrix();

        const auto& cam_pt   = observable.in_camera[feature_id];
        const auto point_ocv = util::ros2opencv_position(cam_pt);
        const auto depth     = point_ocv.z();
        constexpr auto kMinZ = 0.1;
        if (depth < kMinZ) return ObsJacobian::Zero();

        Eigen::Matrix<double, 2, 3> proj_jac;
        // clang-format off
        proj_jac <<
            camera_feature.camera_matrix[0][0] / depth,                        0, -camera_feature.camera_matrix[0][0] * point_ocv.x() / (depth * depth),
                                                  0, camera_feature.camera_matrix[1][1] / depth, -camera_feature.camera_matrix[1][1] * point_ocv.y() / (depth * depth);
        // clang-format on

        Eigen::Matrix3d cam_ros_jac;
        // clang-format off
        cam_ros_jac <<  0, -1,  0,
                        0,  0, -1,
                        1,  0,  0;
        // clang-format on

        Eigen::Matrix<double, 3, 6> pos_jac;
        pos_jac.setZero();
        pos_jac(0, kX) = 1.0;
        pos_jac(1, kY) = 1.0;
        pos_jac(2, kZ) = 1.0;

        const auto r_face = Eigen::AngleAxisd { psi, Eigen::Vector3d::UnitZ() }.toRotationMatrix();

        Eigen::Matrix3d d_r_face_d_psi;
        // clang-format off
        d_r_face_d_psi <<
            -std::sin(psi), -std::cos(psi), 0,
             std::cos(psi), -std::sin(psi), 0,
                        0,              0, 0;
        // clang-format on

        Eigen::Vector3d local_point = Eigen::Vector3d::Zero();

        if (feature_id == Observable::kFeatureIcon) {
            local_point = Eigen::Vector3d(-kRuneIconProminentDistance, 0, 0);
        } else {
            static constexpr std::array kBladeDeg = { 0.0, 72.0, 144.0, 216.0, 288.0 };
            const auto alpha = theta + util::deg2rad(kBladeDeg[feature_id - 1]);
            local_point      = Eigen::Vector3d(
                0, -kRuneGlobalRadius * std::sin(alpha), kRuneGlobalRadius * std::cos(alpha));
            Eigen::Vector3d d_local_d_theta(
                0, -kRuneGlobalRadius * std::cos(alpha), -kRuneGlobalRadius * std::sin(alpha));
            pos_jac.col(kA) = r_face * d_local_d_theta;
        }
        pos_jac.col(kPsi) = d_r_face_d_psi * local_point;

        ObsJacobian J;
        J.noalias() = proj_jac * cam_ros_jac * r_cam_from_odom * pos_jac;
        return J;
    }

    // ===== Single EKF Update =====

    auto update(const Eigen::Vector2d& observed, int feature_id) noexcept -> bool {
        const auto prior_state      = context.posteriors_state;
        const auto prior_covariance = context.posteriors_covariance;
        const auto pred             = observable.projected[feature_id];

        auto innovation = Eigen::Vector2d { observed - pred };
        if (innovation.hasNaN()) return false;

        auto jacobian = make_observation_jacobian(feature_id);
        if (jacobian.hasNaN()) return false;

        ObservationNoise noise_obs = ObservationNoise::Identity() * config.noise_observation;

        auto innovation_covariance = jacobian * prior_covariance * jacobian.transpose() + noise_obs;
        auto innovation_cov_inv    = innovation_covariance.inverse();
        if (innovation_cov_inv.hasNaN()) return false;

        KalmanGain kalman_gain = prior_covariance * jacobian.transpose() * innovation_cov_inv;

        auto posterior_state = StateVector { prior_state };
        posterior_state.noalias() += kalman_gain * innovation;

        if (posterior_state.hasNaN() || !posterior_state.allFinite()) return false;
        posterior_state[kPsi] = util::normalize_angle(posterior_state[kPsi]);

        auto complement                = Covariance::Identity() - kalman_gain * jacobian;
        auto posterior_covariance      = Covariance { };
        posterior_covariance.noalias() = complement * prior_covariance * complement.transpose();
        posterior_covariance += (kalman_gain * noise_obs * kalman_gain.transpose()).eval();
        posterior_covariance = 0.5 * (posterior_covariance + posterior_covariance.transpose());

        context.posteriors_state      = posterior_state;
        context.posteriors_covariance = posterior_covariance;

        observable.update(posterior_state);
        return true;
    }

    // ===== Init =====

    auto init(std::span<const RuneIcon> icons, std::span<const RuneBullseye> bullseyes) noexcept
        -> bool {
        // fprintf(stderr, "fuck\n");

        auto inactive_bullseyes = std::vector<const RuneBullseye*> { };
        for (const auto& bullseye : bullseyes) {
            if (!bullseye.active) inactive_bullseyes.push_back(&bullseye);
        }

        if (inactive_bullseyes.empty()) return false;
        if (inactive_bullseyes.size() > 2) return false;
        if (icons.empty()) return false;

        auto best_candidate = std::optional<InitCandidate> { };
        for (const auto& icon : icons) {
            for (const auto* seed : inactive_bullseyes) {
                if (!seed) continue;

                auto inactive_storage = std::array<RuneBullseye, 2> { };
                for (const auto& [index, item] : inactive_bullseyes | std::views::enumerate) {
                    inactive_storage[static_cast<std::size_t>(index)] = *item;
                }
                const auto inactive = std::span<const RuneBullseye> { inactive_storage.data(),
                    inactive_bullseyes.size() };

                auto candidate = make_init_candidate(icon, *seed, inactive);
                if (!candidate) continue;
                if (!best_candidate || better_init_candidate(*candidate, *best_candidate)) {
                    best_candidate = *candidate;
                }
            }
        }

        if (!best_candidate) return false;
        fprintf(stderr, "fuck yes\n");
        // if (best_candidate->inactive_inliers != static_cast<int>(inactive_bullseyes.size())) {
        //     return false;
        // }

        context.posteriors_state       = StateVector::Zero();
        context.posteriors_state[kX]   = best_candidate->center_odom.x();
        context.posteriors_state[kY]   = best_candidate->center_odom.y();
        context.posteriors_state[kZ]   = best_candidate->center_odom.z();
        context.posteriors_state[kW]   = 0.0;
        context.posteriors_state[kA]   = 0.0;
        context.posteriors_state[kPsi] = best_candidate->face_yaw_odom;

        context.reset_covariance();
        init_timestamp = Clock::now();
        update_count   = 0;
        fitter.reset();

        observable.update(context.posteriors_state);

        return true;
    }

    // ===== Predict =====

    auto predict(double dt, Timestamp stamp) noexcept -> void {
        current_stamp = stamp;

        const auto prior_state      = predict_state(dt, context.posteriors_state);
        const auto prior_covariance = predict_covariance(dt, context.posteriors_covariance);

        context.posteriors_state      = prior_state;
        context.posteriors_covariance = prior_covariance;

        observable.update(prior_state);
    }

    // ===== Correct =====

    auto correct(std::span<const RuneIcon> icons, std::span<const RuneBullseye> bullseyes) noexcept
        -> void {
        observable.update(context.posteriors_state);
        std::ranges::fill(blade_inactive, false);
        addition.predicted.clear();
        for (int feature_id = 0; feature_id < Observable::kFeatureCount; ++feature_id) {
            if (!observable.visible[feature_id]) continue;
            addition.predicted.push_back({
                feature_id,
                Point2d {
                    observable.projected[feature_id].x(), observable.projected[feature_id].y() },
            });
        }

        struct Obs {
            Eigen::Vector2d pixel;
            bool is_icon;
            bool is_inactive;
        };
        auto observations = std::vector<Obs> { };
        for (const auto& ic : icons)
            observations.push_back({ Eigen::Vector2d(ic.center.x, ic.center.y), true, false });
        for (const auto& bs : bullseyes)
            observations.push_back(
                { Eigen::Vector2d(bs.center.x, bs.center.y), false, !bs.active });

        if (observations.empty()) return;
        const auto N = static_cast<int>(observations.size());
        const auto M = Observable::kFeatureCount;

        struct PredInfo {
            ObsJacobian H;
            ObservationNoise S;
        };
        auto pred_infos = std::array<PredInfo, M> { };

        ObservationNoise R = ObservationNoise::Identity() * config.noise_observation;
        const auto& P      = context.posteriors_covariance;

        for (int j = 0; j < M; ++j) {
            pred_infos[j].H = make_observation_jacobian(j);
            pred_infos[j].S = pred_infos[j].H * P * pred_infos[j].H.transpose() + R;
        }

        // Build cost matrix with Mahalanobis distance. Rejects are handled by Hungarian dummies.
        constexpr auto kInf = std::numeric_limits<double>::max() / 4;
        auto cost           = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>(N, M);
        cost.setConstant(kInf);

        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < M; ++j) {
                if (!observable.visible[j]) continue;

                bool mismatch = observations[i].is_icon ? (j != Observable::kFeatureIcon)
                                                        : (j == Observable::kFeatureIcon);
                if (mismatch) continue;

                auto innovation = observations[i].pixel - observable.projected[j];
                auto d2         = util::mahalanobis_distance(innovation, pred_infos[j].S);
                if (!d2) continue;
                cost(i, j) = *d2;
            }
        }

        auto assignments = util::hungarian_assign(cost, config.gate_threshold);

        addition.tracked.clear();
        auto corrected = false;
        for (int i = 0; i < N; ++i) {
            if (!assignments[i]) continue;
            auto j = *assignments[i];
            if (j < 0 || j >= M) continue;

            // Re-evaluate innovation covariance with the freshest state before correcting.
            auto innovation = observations[i].pixel - observable.projected[j];
            auto d2         = util::mahalanobis_distance(innovation,
                pred_infos[j].H * context.posteriors_covariance * pred_infos[j].H.transpose() + R);
            if (!d2) continue;

            if (!update(observations[i].pixel, j)) continue;
            corrected = true;
            if (!observations[i].is_icon && j != Observable::kFeatureIcon) {
                blade_inactive[static_cast<std::size_t>(j - 1)] = observations[i].is_inactive;
            }
            addition.tracked.push_back(
                { j, Point2d { observations[i].pixel.x(), observations[i].pixel.y() } });

            // Refresh pred_infos after state update
            for (int jj = 0; jj < M; ++jj) {
                pred_infos[jj].H = make_observation_jacobian(jj);
                pred_infos[jj].S =
                    pred_infos[jj].H * context.posteriors_covariance * pred_infos[jj].H.transpose()
                    + R;
            }
        }

        if (corrected) {
            update_count += 1;

            auto state   = context.get_state(blade_inactive);
            double t_sec = std::chrono::duration<double>(current_stamp.time_since_epoch()).count();
            fitter.push(t_sec, state.rotation_angle);

            auto& res_linear = fitter.fit_linear();
            auto& res_sine   = fitter.fit_sine();

            if (res_sine.valid && (!res_linear.valid || res_sine.cost < res_linear.cost)) {
                double dt_now                = t_sec - fitter.base_t();
                context.use_prediction_speed = false;
                context.sine_cost            = res_sine.cost;
                context.sine_C               = res_sine.C;
                context.sine_v               = res_sine.v;
                context.sine_a               = res_sine.a;
                context.sine_omega           = res_sine.omega;
                context.sine_phase           = res_sine.omega * dt_now + res_sine.phi;
                context.sine_t               = dt_now;
                context.sine_valid           = true;

                static auto* dump          = fopen("/tmp/rune_sine_dump.csv", "w");
                static bool header_written = false;
                if (dump) {
                    if (!header_written) {
                        fprintf(dump, "t_rel,ekf_theta,C,v,a,omega,phi,cost\n");
                        header_written = true;
                    }
                    fprintf(dump, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                        t_sec - fitter.base_t(), state.rotation_angle, res_sine.C, res_sine.v,
                        res_sine.a, res_sine.omega, res_sine.phi, res_sine.cost);
                }
            } else if (res_linear.valid) {
                context.prediction_speed     = res_linear.speed;
                context.use_prediction_speed = true;
                context.prediction_cost      = res_linear.cost;
                context.sine_valid           = false;
            }
        }
    }

    // ===== Convergence / Divergence =====

    auto converge() const -> bool {
        // constexpr auto kMinUpdate = std::size_t { 10 };
        // constexpr auto kMaxCovXY  = 1.0;
        // constexpr auto kMaxCovYaw = 0.002;

        // if (update_count < kMinUpdate) return false;

        // const auto& cov = context.posteriors_covariance;
        // if (cov(kX, kX) > kMaxCovXY) return false;
        // if (cov(kY, kY) > kMaxCovXY) return false;
        // if (cov(kPsi, kPsi) > kMaxCovYaw) return false;

        // using namespace std::chrono_literals;
        // if (Clock::now() - init_timestamp < 0.1s) return false;

        return true;
    }

    auto diverged() const -> bool {
        const auto& state = context.posteriors_state;
        const auto& cov   = context.posteriors_covariance;

        auto checks = std::array {
            cov(kX, kX) > 150.0,
            cov(kY, kY) > 150.0,
            std::abs(state[kX]) > 15.0,
            std::abs(state[kY]) > 15.0,
            std::abs(state[kZ]) > 2.0,
            std::abs(state[kW]) > 10.0 * std::numbers::pi,
            state.hasNaN() == true,
            state.allFinite() == false,
        };
        return std::ranges::any_of(checks, std::identity { });
    }
};

// ===== PIMPL Boilerplate =====

RuneModel::RuneModel(const Config& cfg) noexcept
    : pimpl { std::make_unique<Impl>(cfg) } { }

RuneModel::~RuneModel() noexcept = default;

auto RuneModel::update_camera(std::array<double, 9> matrix, std::array<double, 5> coeff) noexcept
    -> void {
    pimpl->configure_camera(matrix, coeff);
}

auto RuneModel::update_transform(const Transform& t) noexcept -> void {
    pimpl->update_transform(t);
}

auto RuneModel::init(
    std::span<const RuneIcon> icons, std::span<const RuneBullseye> bullseyes) noexcept -> bool {
    return pimpl->init(icons, bullseyes);
}

auto RuneModel::predict(double dt, Timestamp now) noexcept -> void { pimpl->predict(dt, now); }

auto RuneModel::correct(
    std::span<const RuneIcon> icons, std::span<const RuneBullseye> bullseyes) noexcept -> void {
    pimpl->correct(icons, bullseyes);
}

auto RuneModel::converge() const -> bool { return pimpl->converge(); }
auto RuneModel::diverged() const -> bool { return pimpl->diverged(); }

auto RuneModel::state() const noexcept -> State {
    return pimpl->context.get_state(pimpl->blade_inactive);
}

auto RuneModel::addition() const -> const Addition& { return pimpl->addition; }

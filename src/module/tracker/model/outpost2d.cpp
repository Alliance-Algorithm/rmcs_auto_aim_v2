#include "outpost2d.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/conversion.hpp"
#include "utility/math/solve_pnp/pnp_solution.hpp"
#include "utility/robot/constant.hpp"

#include <opencv2/calib3d.hpp>

using namespace rmcs;

namespace {

constexpr auto kRadius     = kOutpostRadius;
constexpr auto kPitch      = kPredictedOutpostArmorPitch;
constexpr auto kHeightStep = kOutpostArmorHeightStep;
constexpr auto kAngleStep  = 2.0 * std::numbers::pi / 3.0;

}

auto Outpost2dModel::State::transition(double seconds) -> void {
    rotation_angle = util::normalize_angle(rotation_angle + rotation_speed * seconds);
}

auto Outpost2dModel::State::get_direction() const -> Point3d { return Point3d { x, y, z }; }

auto Outpost2dModel::State::get_rotation_speed() const -> double { return rotation_speed; }

auto Outpost2dModel::State::get_aimpoints() const -> std::vector<Point3d> {
    auto result = std::vector<Point3d> { };
    for (int i = 0; i < 3; ++i) {
        const auto offset = i - static_cast<int>(index);
        const auto angle  = rotation_angle + offset * kAngleStep;
        result.emplace_back(
            x + kRadius * std::cos(angle), y + kRadius * std::sin(angle), z - offset * kHeightStep);
    }
    return result;
}

struct Outpost2dModel::Impl {
    using StateVector         = Eigen::Matrix<double, 5, 1>;
    using Covariance          = Eigen::Matrix<double, 5, 5>;
    using ObservationVector   = Eigen::Matrix<double, 8, 1>;
    using ObservationJacobian = Eigen::Matrix<double, 8, 5>;

    static constexpr auto kStateX = 0;
    static constexpr auto kStateY = 1;
    static constexpr auto kStateZ = 2;
    static constexpr auto kStateW = 3;
    static constexpr auto kStateA = 4;

    struct Hypothesis {
        StateVector posteriors_state     = StateVector::Zero();
        Covariance posteriors_covariance = Covariance::Identity();

        // 相位、高度始终对应初始化的参考板，切板只改变 current_index。
        int reference_index       = 0;
        int current_index         = 0;
        int direction             = 0;
        double last_yaw           = 0.0;
        double score              = 0.0;
        std::size_t prune_count   = 0;
        std::size_t correct_count = 0;

        auto get_state() const noexcept -> State {
            return State {
                .x              = posteriors_state[kStateX],
                .y              = posteriors_state[kStateY],
                .z              = posteriors_state[kStateZ],
                .rotation_speed = posteriors_state[kStateW],
                .rotation_angle = posteriors_state[kStateA],
                .index          = static_cast<std::uint8_t>(reference_index),
            };
        }
    };

    struct Observable {
        CameraFeature feature;

        std::array<Eigen::Vector2d, 6> upper2d;
        std::array<Eigen::Vector2d, 6> lower2d;
        std::array<Eigen::Vector3d, 6> upper3d;
        std::array<Eigen::Vector3d, 6> lower3d;
        std::array<bool, 6> visible;

        double yaw_full_max = 70.0 * std::numbers::pi / 180.0;
        double yaw_part_max = 90.0 * std::numbers::pi / 180.0;

        auto update(const Eigen::Vector3d& center, double theta, int reference_index) {
            const auto q_cam_from_odom = feature.orientation.make<Eigen::Quaterniond>().conjugate();
            const auto cam_position    = feature.translation.make<Eigen::Vector3d>();

            std::ranges::fill(upper2d, Eigen::Vector2d::Zero());
            std::ranges::fill(lower2d, Eigen::Vector2d::Zero());
            std::ranges::fill(upper3d, Eigen::Vector3d::Zero());
            std::ranges::fill(lower3d, Eigen::Vector3d::Zero());
            std::ranges::fill(visible, false);

            for (int i = 0; i < 3; ++i) {
                const auto offset   = i - reference_index;
                const auto angle    = theta + offset * kAngleStep;
                const auto position = Eigen::Vector3d {
                    center.x() + kRadius * std::cos(angle),
                    center.y() + kRadius * std::sin(angle),
                    center.z() - offset * kHeightStep,
                };
                const auto orientation = Eigen::Quaterniond {
                    Eigen::AngleAxisd { angle + std::numbers::pi, Eigen::Vector3d::UnitZ() }
                        * Eigen::AngleAxisd { kPitch, Eigen::Vector3d::UnitY() },
                };
                const auto y_axis   = Eigen::Vector3d { orientation * Eigen::Vector3d::UnitY() };
                const auto z_axis   = Eigen::Vector3d { orientation * Eigen::Vector3d::UnitZ() };
                const auto left_id  = i * 2;
                const auto right_id = left_id + 1;
                for (int side = 0; side < 2; ++side) {
                    const auto id         = left_id + side;
                    const auto bar_center = Eigen::Vector3d {
                        position + (side == 0 ? +0.5 : -0.5) * kSmallArmorWidth * y_axis,
                    };
                    // 与 RobotModel 一致，3d 端点缓存使用相机 ROS 坐标系。
                    upper3d[id] = q_cam_from_odom
                        * (bar_center + 0.5 * kLightBarHeight * z_axis - cam_position);
                    lower3d[id] = q_cam_from_odom
                        * (bar_center - 0.5 * kLightBarHeight * z_axis - cam_position);
                }
                constexpr auto kMinDepth = 0.1;
                if (upper3d[left_id].x() < kMinDepth || lower3d[left_id].x() < kMinDepth
                    || upper3d[right_id].x() < kMinDepth || lower3d[right_id].x() < kMinDepth)
                    continue;

                auto points_camera = std::vector<cv::Point3d> { };
                for (const auto& point :
                    { upper3d[left_id], upper3d[right_id], lower3d[right_id], lower3d[left_id] }) {
                    const auto optical = util::ros2opencv_position(point);
                    points_camera.emplace_back(optical.x(), optical.y(), optical.z());
                }
                auto projected  = std::vector<cv::Point2d> { };
                const auto zero = cv::Vec3d { 0.0, 0.0, 0.0 };
                cv::projectPoints(points_camera, zero, zero, feature.intrinsic(),
                    feature.distortion(), projected);
                upper2d[left_id] << projected[0].x, projected[0].y;
                upper2d[right_id] << projected[1].x, projected[1].y;
                lower2d[right_id] << projected[2].x, projected[2].y;
                lower2d[left_id] << projected[3].x, projected[3].y;

                const auto armor_face =
                    Eigen::Vector3d { -(orientation * Eigen::Vector3d::UnitX()) };
                const auto armor_to_cam =
                    Eigen::Vector3d { (cam_position - position).normalized() };
                const auto yaw = std::acos(std::clamp(armor_face.dot(armor_to_cam), -1.0, 1.0));
                if (yaw >= yaw_part_max) continue;
                if (yaw < yaw_full_max) {
                    visible[left_id]  = true;
                    visible[right_id] = true;
                } else {
                    const auto on_plus_y                    = armor_to_cam.dot(y_axis) > 0.0;
                    visible[on_plus_y ? right_id : left_id] = true;
                }
            }
        }

        auto visible_lightbars() const noexcept {
            auto result = std::vector<int> { };
            for (int i = 0; i < 6; ++i)
                if (visible[i]) result.push_back(i);
            return result;
        }

        auto visible_armors() const noexcept {
            auto result = std::vector<int> { };
            for (int i = 0; i < 3; ++i)
                if (visible[i * 2] && visible[i * 2 + 1]) result.push_back(i);
            return result;
        }
    };

    struct Correction {
        StateVector state;
        Covariance covariance;
        double cost;
        double yaw;
        int index;
        std::vector<Addition::Tracked> tracked;
    };

    Config config;
    CameraFeature camera_feature;
    Observable observable;
    Covariance noise_process = Covariance::Zero();
    Covariance noise_pose    = Covariance::Zero();
    std::vector<Hypothesis> hypotheses;
    Armor2d metadata { };
    Addition addition;

    double elapsed           = 0.0;
    double since_correct     = 0.0;
    std::size_t update_count = 0;

    explicit Impl(const Config& cfg) noexcept
        : config { cfg } {
        noise_process.diagonal() << cfg.process_noise_xy, cfg.process_noise_xy, cfg.process_noise_z,
            cfg.process_noise_speed, cfg.process_noise_angle;
        noise_pose.diagonal() << cfg.observation_noise_xy, cfg.observation_noise_xy,
            cfg.observation_noise_z, 0.0, cfg.observation_noise_yaw;
    }

    auto best() const noexcept -> const Hypothesis& {
        return *std::ranges::min_element(hypotheses, { }, &Hypothesis::score);
    }

    auto update_observable(const Hypothesis& model) -> void {
        const auto& state = model.posteriors_state;
        observable.update(state.head<3>(), state[kStateA], model.reference_index);
    }

    static auto make_observation(const Observable& observed, int index) -> ObservationVector {
        const auto left_id  = index * 2;
        const auto right_id = left_id + 1;
        auto points         = ObservationVector { };
        points << observed.upper2d[left_id], observed.upper2d[right_id], observed.lower2d[right_id],
            observed.lower2d[left_id];
        return points;
    }

    static auto make_observation_jacobian(
        const Hypothesis& model, const Observable& observed, int index) -> ObservationJacobian {
        const auto left_id       = index * 2;
        const auto right_id      = left_id + 1;
        const auto points_camera = std::array {
            observed.upper3d[left_id],
            observed.upper3d[right_id],
            observed.lower3d[right_id],
            observed.lower3d[left_id],
        };
        const auto& feature = observed.feature;
        const auto camera_rotation =
            feature.orientation.make<Eigen::Quaterniond>().conjugate().toRotationMatrix();
        const auto camera_position = feature.translation.make<Eigen::Vector3d>();
        const auto center          = Eigen::Vector3d { model.posteriors_state.head<3>() };
        const auto world_to_optical =
            Eigen::Matrix3d { util::kCoordTransformMatrix.transpose() * camera_rotation };

        // 由端点缓存单独求投影雅可比，保留畸变项，与 update() 的像素投影一致。
        auto optical_points = std::vector<cv::Point3d> { };
        for (const auto& point : points_camera) {
            const auto optical = util::ros2opencv_position(point);
            optical_points.emplace_back(optical.x(), optical.y(), optical.z());
        }
        auto projected           = std::vector<cv::Point2d> { };
        auto projection_jacobian = cv::Mat { };
        const auto zero          = cv::Vec3d { 0.0, 0.0, 0.0 };
        cv::projectPoints(optical_points, zero, zero, feature.intrinsic(), feature.distortion(),
            projected, projection_jacobian);

        auto jacobian = ObservationJacobian { };
        for (int i = 0; i < 4; ++i) {
            const auto point_world = Eigen::Vector3d {
                camera_rotation.transpose() * points_camera[i] + camera_position,
            };
            auto position_jacobian = Eigen::Matrix<double, 3, 5>::Zero().eval();
            position_jacobian.block<3, 3>(0, kStateX).setIdentity();
            position_jacobian.col(kStateA) = Eigen::Vector3d::UnitZ().cross(point_world - center);

            auto pixel_jacobian = Eigen::Matrix<double, 2, 3> { };
            for (int row = 0; row < 2; ++row) {
                for (int col = 0; col < 3; ++col) {
                    pixel_jacobian(row, col) = projection_jacobian.at<double>(2 * i + row, 3 + col);
                }
            }
            jacobian.block<2, 5>(2 * i, 0) = pixel_jacobian * world_to_optical * position_jacobian;
        }
        return jacobian;
    }

    auto full() const -> std::array<Armor3d, 3> {
        auto result = std::array<Armor3d, 3> { };
        if (hypotheses.empty()) return result;

        const auto state     = best().get_state();
        const auto positions = state.get_aimpoints();
        for (int i = 0; i < 3; ++i) {
            const auto angle       = state.rotation_angle + (i - state.index) * kAngleStep;
            const auto orientation = Eigen::Quaterniond {
                Eigen::AngleAxisd { angle + std::numbers::pi, Eigen::Vector3d::UnitZ() }
                    * Eigen::AngleAxisd { kPitch, Eigen::Vector3d::UnitY() },
            };
            result[i] = Armor3d {
                .genre       = metadata.genre,
                .color       = metadata.color,
                .id          = i,
                .translation = Translation { positions[i] },
                .orientation = Orientation { orientation },
            };
        }
        return result;
    }

    auto update_addition() -> void {
        if (hypotheses.empty()) return;
        update_observable(best());
        for (int i = 0; i < 3; ++i) {
            auto& armor         = addition.armors[i];
            armor               = metadata;
            const auto left_id  = i * 2;
            const auto right_id = left_id + 1;
            armor.tl            = Point2d { observable.upper2d[left_id] }.make<cv::Point2f>();
            armor.tr            = Point2d { observable.upper2d[right_id] }.make<cv::Point2f>();
            armor.br            = Point2d { observable.lower2d[right_id] }.make<cv::Point2f>();
            armor.bl            = Point2d { observable.lower2d[left_id] }.make<cv::Point2f>();
            armor.center        = (armor.tl + armor.tr + armor.br + armor.bl) * 0.25f;
        }
    }

    auto configure_camera(std::array<double, 9> matrix, std::array<double, 5> coeff) noexcept {
        camera_feature.from(matrix);
        camera_feature.from(coeff);
        observable.feature.from(matrix);
        observable.feature.from(coeff);
        update_addition();
    }

    auto update_transform(const Transform& transform) noexcept -> void {
        camera_feature.translation     = transform.translation;
        camera_feature.orientation     = transform.orientation;
        observable.feature.translation = transform.translation;
        observable.feature.orientation = transform.orientation;
        update_addition();
    }

    auto refine_seed(const StateVector& state, const ObservationVector& points) const
        -> std::optional<std::pair<StateVector, double>> {
        auto seed             = Hypothesis { };
        seed.posteriors_state = state;
        auto observed         = Observable { };
        observed.feature      = camera_feature;
        observed.update(state.head<3>(), state[kStateA], seed.reference_index);
        if (!observed.visible[0] || !observed.visible[1]) return std::nullopt;
        auto error = (points - make_observation(observed, 0)).squaredNorm();
        for (int iteration = 0; iteration < 10; ++iteration) {
            const auto jacobian = make_observation_jacobian(seed, observed, 0);
            auto normal         = Covariance { jacobian.transpose() * jacobian };
            normal.diagonal().array() += 1e-3;
            const auto increment = StateVector {
                normal.ldlt().solve(
                    jacobian.transpose() * (points - make_observation(observed, 0))),
            };
            if (!increment.allFinite()) break;

            auto improved = false;
            for (const auto scale : { 1.0, 0.5, 0.25, 0.125 }) {
                auto trial = seed;
                trial.posteriors_state += scale * increment;
                auto projected    = Observable { };
                projected.feature = camera_feature;
                projected.update(trial.posteriors_state.head<3>(), trial.posteriors_state[kStateA],
                    trial.reference_index);
                if (!projected.visible[0] || !projected.visible[1]) continue;
                const auto trial_error = (points - make_observation(projected, 0)).squaredNorm();
                if (trial_error >= error) continue;
                seed     = trial;
                observed = std::move(projected);
                error    = trial_error;
                improved = true;
                break;
            }
            if (!improved) break;
        }
        seed.posteriors_state[kStateA] = util::normalize_angle(seed.posteriors_state[kStateA]);
        return std::pair { seed.posteriors_state, error };
    }

    auto init(const Armor2d& armor) noexcept -> bool {
        auto pnp                      = util::RobustPnpSolution { };
        pnp.input.feature             = camera_feature;
        pnp.input.armor2d             = armor;
        pnp.input.fixed_outpost_pitch = true;
        if (!pnp.solve()) return false;

        const auto& armor3d        = pnp.result.armor3d;
        const auto orientation     = armor3d.orientation.make<Eigen::Quaterniond>();
        const auto translation     = armor3d.translation.make<Eigen::Vector3d>();
        const auto armor_to_center = Eigen::Vector3d {
            Eigen::AngleAxisd { -kPitch, orientation * Eigen::Vector3d::UnitY() }
                * (orientation * Eigen::Vector3d::UnitX()),
        };
        const auto pnp_angle =
            util::normalize_angle(std::atan2(-armor_to_center.y(), -armor_to_center.x()));

        // 单板 PnP 的镜像解可能在固定 pitch 后反而更差，用同一组四点细化初始相位。
        auto points = ObservationVector { };
        points << armor.tl.x, armor.tl.y, armor.tr.x, armor.tr.y, armor.br.x, armor.br.y,
            armor.bl.x, armor.bl.y;
        const auto camera_position = camera_feature.translation.make<Eigen::Vector3d>();
        const auto facing          = std::atan2(
            camera_position.y() - translation.y(), camera_position.x() - translation.x());
        auto best_seed = std::optional<std::pair<StateVector, double>> { };
        for (const auto yaw : { pnp_angle, util::normalize_angle(2.0 * facing - pnp_angle) }) {
            auto state = StateVector { };
            state << translation.x() - kRadius * std::cos(yaw),
                translation.y() - kRadius * std::sin(yaw), translation.z(), 0.0, yaw;
            const auto seed = refine_seed(state, points);
            if (!seed || (best_seed && seed->second >= best_seed->second)) continue;
            best_seed = seed;
        }
        if (!best_seed) return false;
        const auto center = Eigen::Vector3d { best_seed->first.head<3>() };
        const auto angle  = best_seed->first[kStateA];

        hypotheses.clear();
        for (const auto direction : { +1, -1, 0 }) {
            // 静止时单板的绝对高度不可辨，约定以中板为参考。
            const auto count = direction == 0 ? 1 : 3;
            for (int i = 0; i < count; ++i) {
                auto model            = Hypothesis { };
                model.reference_index = direction == 0 ? 1 : i;
                model.current_index   = model.reference_index;
                model.direction       = direction;
                model.last_yaw        = angle;
                model.posteriors_state << center.x(), center.y(), center.z(),
                    direction * kOutpostAngularSpeed, angle;
                model.posteriors_covariance.diagonal() << 1.0, 1.0, 0.04,
                    direction == 0 ? 0.0 : 0.04, 0.4;
                hypotheses.push_back(model);
            }
        }
        metadata      = armor;
        elapsed       = 0.0;
        since_correct = 0.0;
        update_count  = 0;
        addition.tracked.clear();
        update_addition();
        return true;
    }

    auto predict(double dt) noexcept -> void {
        elapsed += dt;
        since_correct += dt;
        for (auto& model : hypotheses) {
            auto& state    = model.posteriors_state;
            state[kStateA] = util::normalize_angle(state[kStateA] + state[kStateW] * dt);

            auto jacobian              = Covariance { Covariance::Identity() };
            jacobian(kStateA, kStateW) = dt;
            auto noise                 = Covariance { noise_process * dt };
            if (model.direction == 0) {
                noise(kStateW, kStateW) = 0.0;
                noise(kStateA, kStateA) = 0.0;
            } else {
                const auto speed_noise = config.process_noise_speed;
                noise(kStateA, kStateA) += speed_noise * dt * dt * dt / 3.0;
                noise(kStateA, kStateW) = noise(kStateW, kStateA) = speed_noise * dt * dt / 2.0;
            }
            model.posteriors_covariance =
                (jacobian * model.posteriors_covariance * jacobian.transpose() + noise).eval();
        }
        addition.tracked.clear();
        update_addition();
    }

    template <int Size>
    auto evaluate(const Hypothesis& model, const Eigen::Matrix<double, Size, 1>& innovation,
        const Eigen::Matrix<double, Size, 5>& jacobian, int index) const
        -> std::optional<Correction> {
        using ObservationNoise = Eigen::Matrix<double, Size, Size>;
        using KalmanGain       = Eigen::Matrix<double, 5, Size>;
        const auto& covariance = model.posteriors_covariance;
        // Config 保持米/弧度的语义，投影到像素域后再加角点自身的像素方差。
        const auto noise = ObservationNoise {
            jacobian * noise_pose * jacobian.transpose() + ObservationNoise::Identity(),
        };
        const auto innovation_covariance = ObservationNoise {
            jacobian * covariance * jacobian.transpose() + noise,
        };
        const auto decomposition = innovation_covariance.ldlt();
        if (!decomposition.isPositive()) return std::nullopt;
        const auto distance  = innovation.dot(decomposition.solve(innovation));
        constexpr auto kGate = 10.0 * Size;
        if (!std::isfinite(distance) || distance > kGate) return std::nullopt;

        const auto gain = KalmanGain {
            decomposition.solve((covariance * jacobian.transpose()).transpose()).transpose(),
        };
        auto state = StateVector { model.posteriors_state + gain * innovation };
        if (!state.allFinite()) return std::nullopt;
        state[kStateA] = util::normalize_angle(state[kStateA]);
        // 相位修正超过半个板间隔，应重新关联，不能靠相位跳变来解释切板。
        if (std::abs(util::normalize_angle(state[kStateA] - model.posteriors_state[kStateA]))
            > 0.5 * kAngleStep)
            return std::nullopt;
        if (model.direction == 0) {
            state[kStateW] = 0.0;
        } else {
            // 分支不能靠变号或退化成零速来解释另一种旋向/静止模型。
            state[kStateW] = model.direction
                * std::clamp(model.direction * state[kStateW], 0.5 * kOutpostAngularSpeed,
                    1.5 * kOutpostAngularSpeed);
        }

        const auto yaw =
            util::normalize_angle(state[kStateA] + (index - model.reference_index) * kAngleStep);
        if (index != model.current_index) {
            const auto threshold = util::deg2rad(config.plate_switch_yaw_min);
            const auto travel    = std::abs(model.posteriors_state[kStateW]) * since_correct;
            // 短间隔内必须有真实的观测朝向跳变；长时间失帧则直接按预测相位重关联。
            if (travel < threshold
                && std::abs(util::normalize_angle(yaw - model.last_yaw)) < threshold)
                return std::nullopt;
        }

        const auto complement     = Covariance { Covariance::Identity() - gain * jacobian };
        auto posterior_covariance = Covariance {
            complement * covariance * complement.transpose() + gain * noise * gain.transpose(),
        };
        posterior_covariance =
            (0.5 * (posterior_covariance + posterior_covariance.transpose())).eval();
        const auto cost = 0.5 * (distance + decomposition.vectorD().array().log().sum());
        return Correction { state, posterior_covariance, cost, yaw, index, { } };
    }

    auto evaluate(const Hypothesis& model, const ObservationVector& points, int index) const
        -> std::optional<Correction> {
        const auto jacobian   = make_observation_jacobian(model, observable, index);
        const auto innovation = ObservationVector { points - make_observation(observable, index) };
        return evaluate(model, innovation, jacobian, index);
    }

    auto correct_lightbars(Correction& correction, const Hypothesis& model, const Armor2d& armor,
        std::span<const Lightbar2d> lightbars) const -> void {
        const auto left_id = correction.index * 2;
        correction.tracked.push_back({ left_id, Point2d { armor.tl } });
        correction.tracked.push_back({ left_id + 1, Point2d { armor.tr } });
        if (lightbars.empty()) return;

        auto posterior                  = model;
        posterior.posteriors_state      = correction.state;
        posterior.posteriors_covariance = correction.covariance;
        posterior.current_index         = correction.index;
        posterior.last_yaw              = correction.yaw;
        auto projected                  = observable;
        projected.update(
            correction.state.head<3>(), correction.state[kStateA], model.reference_index);

        // 完整板锚定后，只关联其余可见灯条。前哨站有高度差，不能仅按像素 x 排序编号。
        auto measured = std::vector<Eigen::Vector4d> {
            { armor.tl.x, armor.tl.y, armor.bl.x, armor.bl.y },
            { armor.tr.x, armor.tr.y, armor.br.x, armor.br.y },
        };
        const auto duplicate = [&](const Eigen::Vector4d& points) {
            const auto tolerance =
                std::max(2.0, 0.15 * (points.head<2>() - points.tail<2>()).norm());
            return std::ranges::any_of(measured, [&](const Eigen::Vector4d& previous) {
                return (points.head<2>() - previous.head<2>()).squaredNorm() < tolerance * tolerance
                    && (points.tail<2>() - previous.tail<2>()).squaredNorm()
                    < tolerance * tolerance;
            });
        };
        const auto match_error = [&](const Eigen::Vector4d& points, int id) {
            const auto predicted =
                Eigen::Vector2d { projected.upper2d[id] - projected.lower2d[id] };
            const auto observed    = Eigen::Vector2d { points.head<2>() - points.tail<2>() };
            const auto length      = predicted.norm();
            const auto gate        = std::max(3.0, 0.5 * length);
            const auto upper_error = (points.head<2>() - projected.upper2d[id]).squaredNorm();
            const auto lower_error = (points.tail<2>() - projected.lower2d[id]).squaredNorm();
            if (observed.norm() < 0.5 * length || observed.norm() > 1.5 * length
                || observed.dot(predicted) < 0.8 * observed.norm() * length
                || upper_error > gate * gate || lower_error > gate * gate)
                return std::numeric_limits<double>::infinity();
            return (upper_error + lower_error) / (gate * gate);
        };
        struct Match {
            Eigen::Vector4d points;
            double error;
            int id;
        };
        auto matches = std::vector<Match> { };
        for (const auto& lightbar : lightbars) {
            if (lightbar.genre != ArmorGenre::UNKNOWN && lightbar.genre != armor.genre) continue;
            if (lightbar.color != ArmorColor::DARK && lightbar.color != armor.color) continue;
            const auto points = Eigen::Vector4d {
                lightbar.upper.x,
                lightbar.upper.y,
                lightbar.lower.x,
                lightbar.lower.y,
            };
            if (!points.allFinite() || duplicate(points)) continue;
            for (const auto id : projected.visible_lightbars()) {
                if (id / 2 == correction.index) continue;
                const auto error = match_error(points, id);
                if (std::isfinite(error)) matches.push_back({ points, error, id });
            }
        }
        std::ranges::sort(matches, [](const auto& a, const auto& b) {
            if (a.error != b.error) return a.error < b.error;
            if (a.id != b.id) return a.id < b.id;
            return std::lexicographical_compare(
                a.points.data(), a.points.data() + 4, b.points.data(), b.points.data() + 4);
        });

        auto used = std::array<bool, 6> { };
        for (const auto& match : matches) {
            const auto id = match.id;
            if (used[id] || duplicate(match.points) || !projected.visible[id]
                || !std::isfinite(match_error(match.points, id)))
                continue;
            const auto armor_jacobian = make_observation_jacobian(posterior, projected, id / 2);
            auto jacobian             = Eigen::Matrix<double, 4, 5> { };
            jacobian.topRows<2>()     = armor_jacobian.middleRows<2>(id % 2 == 0 ? 0 : 2);
            jacobian.bottomRows<2>()  = armor_jacobian.middleRows<2>(id % 2 == 0 ? 6 : 4);
            auto innovation           = Eigen::Vector4d { };
            innovation << match.points.head<2>() - projected.upper2d[id],
                match.points.tail<2>() - projected.lower2d[id];
            const auto refined = evaluate(posterior, innovation, jacobian, correction.index);
            // 与未匹配的固定代价比较，避免只使用完整板的分支因观测更少而占优。
            constexpr auto kUnmatchedCost = 20.0;
            if (!refined || refined->cost >= kUnmatchedCost) continue;
            correction.state      = refined->state;
            correction.covariance = refined->covariance;
            correction.yaw        = refined->yaw;
            correction.cost += refined->cost - kUnmatchedCost;
            correction.tracked.push_back({ id, Point2d { match.points.head<2>() } });
            used[id] = true;
            measured.push_back(match.points);
            posterior.posteriors_state      = correction.state;
            posterior.posteriors_covariance = correction.covariance;
            posterior.last_yaw              = correction.yaw;
            projected.update(
                correction.state.head<3>(), correction.state[kStateA], model.reference_index);
        }
    }

    auto validate_correction(const Hypothesis& model, const Correction& correction,
        const ObservationVector& points) const -> bool {
        const auto& prior = model.posteriors_state;
        const auto& state = correction.state;
        const auto xy_error =
            std::hypot(state[kStateX] - prior[kStateX], state[kStateY] - prior[kStateY]);
        const auto xy_limit = std::hypot(state[kStateX], state[kStateY]) > 5.0 ? 0.20 : 0.10;
        // 初次校正允许修正 PnP 初值，之后沿用旧前哨站的中心跳变门禁。
        if (model.correct_count > 0 && xy_error > xy_limit) return false;

        auto projected = observable;
        projected.update(state.head<3>(), state[kStateA], model.reference_index);
        const auto innovation = ObservationVector {
            points - make_observation(projected, correction.index),
        };
        if (!innovation.allFinite()) return false;

        // 校正后必须实际拟合完整板，不能只凭马氏距离小就刷新有效观测时间。
        const auto height = 0.5
            * ((points.segment<2>(0) - points.segment<2>(6)).norm()
                + (points.segment<2>(2) - points.segment<2>(4)).norm());
        // 用灯条像素尺度换算位置容差，门限不随滤波协方差放大。
        const auto pixel_limit = std::max(3.0, xy_limit * height / kLightBarHeight);
        for (int i = 0; i < 4; ++i) {
            if (innovation.segment<2>(i * 2).squaredNorm() > pixel_limit * pixel_limit)
                return false;
        }
        return true;
    }

    auto correct(const Armor2d& armor, std::span<const Lightbar2d> lightbars) noexcept -> void {
        if (hypotheses.empty()) return;
        auto points = ObservationVector { };
        points << armor.tl.x, armor.tl.y, armor.tr.x, armor.tr.y, armor.br.x, armor.br.y,
            armor.bl.x, armor.bl.y;

        auto corrections = std::vector<std::optional<Correction>> { };
        auto min_cost    = std::numeric_limits<double>::max();
        for (const auto& model : hypotheses) {
            update_observable(model);
            auto candidate = std::optional<Correction> { };
            for (const auto i : observable.visible_armors()) {
                if (model.direction == 0 && i != model.current_index) continue;
                auto correction = evaluate(model, points, i);
                if (!correction) continue;
                correct_lightbars(*correction, model, armor, lightbars);
                if (!validate_correction(model, *correction, points)) continue;
                if (candidate && correction->cost >= candidate->cost) continue;
                candidate = correction;
            }
            if (candidate) min_cost = std::min(min_cost, candidate->cost);
            corrections.push_back(candidate);
        }
        addition.tracked.clear();
        // 全部分支都拒绝的帧不提供剪枝证据，也不刷新 since_correct；持续失败由 tracker 重建。
        if (min_cost == std::numeric_limits<double>::max()) {
            update_addition();
            return;
        }

        for (std::size_t i = 0; i < hypotheses.size(); ++i) {
            auto& model             = hypotheses[i];
            const auto& correction  = corrections[i];
            constexpr auto kMaxCost = 8.0;
            const auto cost =
                correction ? std::min(correction->cost - min_cost, kMaxCost) : kMaxCost;
            model.score = 0.98 * model.score + cost;
            if (!correction) continue;
            model.posteriors_state      = correction->state;
            model.posteriors_covariance = correction->covariance;
            model.current_index         = correction->index;
            model.last_yaw              = correction->yaw;
            model.correct_count += 1;
        }

        const auto best_index = static_cast<std::size_t>(&best() - hypotheses.data());
        if (corrections[best_index]) {
            addition.tracked = std::move(corrections[best_index]->tracked);
        }
        const auto min_score = best().score;
        for (auto& model : hypotheses) {
            model.score -= min_score;
            model.prune_count = model.score > 30.0 ? model.prune_count + 1 : 0;
        }
        update_count += 1;
        // 至少积累一段运动和连续多帧的劣势，不能由某一帧切板/噪声直接删分支。
        if (update_count >= 15 && elapsed >= 0.5) {
            std::erase_if(hypotheses, [](const auto& model) { return model.prune_count >= 5; });
        }
        metadata      = armor;
        since_correct = 0.0;
        update_addition();
    }

    auto converge() const -> bool {
        if (hypotheses.size() != 1 || update_count < 10 || elapsed < 0.2) return false;
        const auto& covariance = best().posteriors_covariance;
        return covariance(kStateX, kStateX) < 0.01 && covariance(kStateY, kStateY) < 0.01
            && covariance(kStateA, kStateA) < 0.01;
    }

    auto diverged() const -> bool {
        if (hypotheses.empty()) return true;
        const auto& state      = best().posteriors_state;
        const auto& covariance = best().posteriors_covariance;
        return std::ranges::any_of(
            std::array {
                since_correct > 0.5,
                covariance(kStateX, kStateX) > 150.0,
                covariance(kStateY, kStateY) > 150.0,
                std::abs(state[kStateX]) > 15.0,
                std::abs(state[kStateY]) > 15.0,
                std::abs(state[kStateW]) > 10.0 * std::numbers::pi,
                !state.allFinite(),
                !covariance.allFinite(),
            },
            std::identity { });
    }
};

Outpost2dModel::Outpost2dModel() noexcept
    : Outpost2dModel { Config { } } { }

Outpost2dModel::Outpost2dModel(const Config& cfg) noexcept
    : pimpl { std::make_unique<Impl>(cfg) } { }

Outpost2dModel::~Outpost2dModel() noexcept = default;

auto Outpost2dModel::update_camera(
    std::array<double, 9> matrix, std::array<double, 5> coeff) noexcept -> void {
    pimpl->configure_camera(matrix, coeff);
}

auto Outpost2dModel::update_transform(const Transform& transform) noexcept -> void {
    pimpl->update_transform(transform);
}

auto Outpost2dModel::init(const Armor2d& armor) noexcept -> bool { return pimpl->init(armor); }

auto Outpost2dModel::predict(double dt) noexcept -> void { pimpl->predict(dt); }

auto Outpost2dModel::correct(const Armor2d& armor, std::span<const Lightbar2d> lightbars) noexcept
    -> void {
    pimpl->correct(armor, lightbars);
}

auto Outpost2dModel::state() noexcept -> State { return pimpl->best().get_state(); }

auto Outpost2dModel::full() const -> std::array<Armor3d, 3> { return pimpl->full(); }

auto Outpost2dModel::current() const -> Armor2d {
    return pimpl->addition.armors[pimpl->best().current_index];
}

auto Outpost2dModel::addition() const -> const Addition& { return pimpl->addition; }

auto Outpost2dModel::converge() const -> bool { return pimpl->converge(); }

auto Outpost2dModel::diverged() const -> bool { return pimpl->diverged(); }

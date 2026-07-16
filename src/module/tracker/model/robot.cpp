#include "robot.hpp"

#include "utility/clock.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/camera.hpp"
#include "utility/math/conversion.hpp"
#include "utility/math/reprojection.hpp"
#include "utility/math/robot.hpp"
#include "utility/math/solve_pnp/pnp_solution.hpp"
#include "utility/robot/constant.hpp"

#include <eigen3/Eigen/Geometry>
#include <ranges>

using namespace rmcs;

auto RobotModel::State::transition(double seconds) -> void {
    x += vx * seconds;
    y += vy * seconds;
    z += vz * seconds;
    rotation_angle = util::normalize_angle(rotation_angle + rotation_speed * seconds);
}

auto RobotModel::State::get_direction() const -> Point3d { return Point3d { x, y, z }; }

auto RobotModel::State::get_rotation_speed() const -> double { return rotation_speed; }

auto RobotModel::State::get_aimpoints() const -> std::vector<Point3d> {
    auto solution = RobotSolution { };

    solution.input.center = Eigen::Vector3d { x, y, z };
    solution.input.toward =
        Eigen::Quaterniond { Eigen::AngleAxisd { rotation_angle, Eigen::Vector3d::UnitZ() } };
    solution.input.radius_forward = radius_forward;
    solution.input.radius_lateral = radius_lateral;
    solution.input.height_lateral = height_lateral;

    const auto armors = solution.solve_armors();
    auto result       = std::vector<Point3d> { };
    for (int i = 0; i < 4; ++i) {
        result.emplace_back(armors[i].translation);
    }
    return result;
}

struct RobotModel::Impl {
    using StateVector      = Eigen::Matrix<double, 11, 1>;
    using Covariance       = Eigen::Matrix<double, 11, 11>;
    using ProcessNoise     = Eigen::Matrix<double, 11, 11>;
    using ObservationNoise = Eigen::Matrix<double, 4, 4>;
    using KalmanGain       = Eigen::Matrix<double, 11, 4>;

    static constexpr auto kStateX  = 0;
    static constexpr auto kStateY  = 1;
    static constexpr auto kStateZ  = 2;
    static constexpr auto kStateVx = 3;
    static constexpr auto kStateVy = 4;
    static constexpr auto kStateVz = 5;
    static constexpr auto kStateA  = 6; // rotation_angle
    static constexpr auto kStateW  = 7; // rotation_speed
    static constexpr auto kStateRF = 8; // radius_forward
    static constexpr auto kStateRL = 9; // radius_lateral
    static constexpr auto kStateHL = 10; // height_lateral

    struct Context {
        StateVector posteriors_state       = StateVector::Zero();
        Covariance posteriors_covariance   = Covariance::Identity();
        ProcessNoise noise_process         = ProcessNoise::Zero();
        ObservationNoise noise_observation = ObservationNoise::Zero();

        auto reset_covariance() noexcept {
            auto diag = StateVector { };
            diag << 64.0, 64.0, 64.0, // x, y, z
                64.0, 64.0, 64.0, // vx, vy, vz
                0.4, // rotation_angle
                100.0, // rotation_speed
                1e-2, 1e-2, 1e-2; // rf, rl, hl
            posteriors_covariance = diag.asDiagonal();
        }

        auto set_noise(const Config& cfg) noexcept {
            noise_process.diagonal() << cfg.noise_x, cfg.noise_y, cfg.noise_z, //
                cfg.noise_vx, cfg.noise_vy, cfg.noise_vz, //
                cfg.noise_rotation_angle, cfg.noise_rotation_speed, //
                cfg.noise_radius_forward, cfg.noise_radius_lateral, cfg.noise_height_lateral;

            noise_observation.diagonal() << cfg.noise_observation, cfg.noise_observation,
                cfg.noise_observation, cfg.noise_observation;
        }

        auto get_state() const noexcept {
            return State {
                .x = posteriors_state[kStateX],
                .y = posteriors_state[kStateY],
                .z = posteriors_state[kStateZ],

                .vx = posteriors_state[kStateVx],
                .vy = posteriors_state[kStateVy],
                .vz = posteriors_state[kStateVz],

                .rotation_angle = posteriors_state[kStateA],
                .rotation_speed = posteriors_state[kStateW],

                .radius_forward = posteriors_state[kStateRF],
                .radius_lateral = posteriors_state[kStateRL],
                .height_lateral = posteriors_state[kStateHL],
            };
        }
    };

    struct Observable {
        util::CameraFeature feature;

        std::array<Eigen::Vector2d, 8> upper2d;
        std::array<Eigen::Vector2d, 8> lower2d;
        std::array<Eigen::Vector3d, 8> upper3d;
        std::array<Eigen::Vector3d, 8> lower3d;
        std::array<bool, 8> visible;

        double yaw_full_max = 60.0 * std::numbers::pi / 180.0;
        double yaw_part_max = 90.0 * std::numbers::pi / 180.0;

        auto update(const Eigen::Vector3d& center, double theta, double rf, double rl, double hl,
            ArmorGenre genre) {
            const auto q_odom_from_cam = feature.orientation.make<Eigen::Quaterniond>();
            const auto q_cam_from_odom = q_odom_from_cam.conjugate();
            const auto cam_position    = feature.translation.make<Eigen::Vector3d>();

            std::ranges::fill(upper2d, Eigen::Vector2d::Zero());
            std::ranges::fill(lower2d, Eigen::Vector2d::Zero());
            std::ranges::fill(upper3d, Eigen::Vector3d::Zero());
            std::ranges::fill(lower3d, Eigen::Vector3d::Zero());
            std::ranges::fill(visible, false);

            auto solution = RobotSolution { };

            solution.input.center = center;
            solution.input.toward =
                Eigen::Quaterniond { Eigen::AngleAxisd { theta, Eigen::Vector3d::UnitZ() } };

            solution.input.radius_forward = rf;
            solution.input.radius_lateral = rl;
            solution.input.height_lateral = hl;
            solution.input.genre          = genre;

            const auto lightbars = solution.solve_lightbars();
            const auto armors    = solution.solve_armors();

            for (int i = 0; i < 8; ++i) {
                const auto upper_odom = lightbars[i].upper.make<Eigen::Vector3d>();
                const auto lower_odom = lightbars[i].lower.make<Eigen::Vector3d>();

                const auto upper_camera = q_cam_from_odom * (upper_odom - cam_position);
                const auto lower_camera = q_cam_from_odom * (lower_odom - cam_position);

                upper3d[i] = upper_camera;
                lower3d[i] = lower_camera;

                const auto upper_opt =
                    reproject_point(Point3d { util::ros2opencv_position(upper_camera) }, feature);
                const auto lower_opt =
                    reproject_point(Point3d { util::ros2opencv_position(lower_camera) }, feature);

                if (upper_opt && lower_opt) {
                    upper2d[i] = upper_opt->make<Eigen::Vector2d>();
                    lower2d[i] = lower_opt->make<Eigen::Vector2d>();
                } else {
                    upper2d[i] = Eigen::Vector2d::Zero();
                    lower2d[i] = Eigen::Vector2d::Zero();
                }
            }

            for (int i = 0; i < 4; ++i) {
                const auto orientation = armors[i].orientation.make<Eigen::Quaterniond>();
                const auto position    = armors[i].translation.make<Eigen::Vector3d>();

                const auto armor_to_center = orientation * Eigen::Vector3d::UnitX();
                const auto armor_face      = -armor_to_center;
                const auto armor_to_cam    = (cam_position - position).normalized();
                const auto yaw = std::acos(std::clamp(armor_face.dot(armor_to_cam), -1.0, 1.0));

                if (yaw >= yaw_part_max) continue;

                const auto left_id  = i * 2;
                const auto right_id = i * 2 + 1;

                if (yaw < yaw_full_max) {
                    visible[left_id]  = true;
                    visible[right_id] = true;
                } else {
                    const auto y_axis = Eigen::Vector3d { orientation * Eigen::Vector3d::UnitY() };
                    const auto on_plus_y                    = armor_to_cam.dot(y_axis) > 0.0;
                    visible[on_plus_y ? right_id : left_id] = true;
                }
            }
        }

        auto visible_lightbars() const noexcept {
            auto result = std::vector<int> { };
            for (int i = 0; i < 8; ++i)
                if (visible[i]) result.push_back(i);
            return result;
        }

        auto visible_armors() const noexcept {
            auto result = std::vector<int> { };
            for (int i = 0; i < 4; ++i)
                if (visible[i * 2] && visible[i * 2 + 1]) result.push_back(i);
            return result;
        }
    };

    Config config { };

    util::CameraFeature camera_feature;

    Context context;
    Observable observable;

    Timestamp init_timestamp = Clock::now();
    std::size_t update_count = 0;
    ArmorGenre genre { DeviceId::UNKNOWN };
    ArmorColor color { ArmorColor::DARK };

    Addition addition { };

private:
    static auto squared_distance(const Eigen::Vector2d& a, const Eigen::Vector2d& b) noexcept {
        const auto dx = a.x() - b.x();
        const auto dy = a.y() - b.y();
        return dx * dx + dy * dy;
    }

    auto make_observation_jacobian(int lightbar_id) const noexcept -> Eigen::Matrix<double, 4, 11> {
        constexpr auto kEpsilon = 1e-4;

        const auto& state = context.posteriors_state;
        if (state.hasNaN()) {
            return Eigen::Matrix<double, 4, 11>::Zero();
        }
        const auto theta = state[kStateA];

        const auto fx = camera_feature.camera_matrix[0][0];
        const auto fy = camera_feature.camera_matrix[1][1];

        const auto r_cam2odom =
            camera_feature.orientation.make<Eigen::Quaterniond>().conjugate().toRotationMatrix();

        auto jacobian = Eigen::Matrix<double, 4, 11> { };
        jacobian.setZero();

        const auto projection_jacobian =
            [=](const Eigen::Vector3d& point) -> std::optional<Eigen::Matrix<double, 2, 3>> {
            constexpr auto kMinZ = 0.1;

            const auto point_camera = util::ros2opencv_position(point);
            const auto depth        = point_camera.z();
            if (depth < kMinZ) return std::nullopt;

            auto jacobian_pixel_camera = Eigen::Matrix<double, 2, 3> { };
            jacobian_pixel_camera << fx / depth, 0, -fx * point_camera.x() / (depth * depth), 0,
                fy / depth, -fy * point_camera.y() / (depth * depth);
            auto jacobian_camera_ros = Eigen::Matrix3d { };
            jacobian_camera_ros << 0, -1, 0, 0, 0, -1, 1, 0, 0;
            return jacobian_pixel_camera * jacobian_camera_ros;
        };

        auto theta_jacobian = [&](int id, bool is_upper) {
            auto compute = [&](double angle) -> Eigen::Vector3d {
                auto solution                 = RobotSolution { };
                solution.input.center         = Translation { Eigen::Vector3d {
                    state[kStateX], state[kStateY], state[kStateZ] } };
                solution.input.toward         = Orientation { Eigen::Quaterniond {
                    Eigen::AngleAxisd { angle, Eigen::Vector3d::UnitZ() } } };
                solution.input.radius_forward = state[kStateRF];
                solution.input.radius_lateral = state[kStateRL];
                solution.input.height_lateral = state[kStateHL];
                solution.input.genre          = genre;
                const auto lightbars          = solution.solve_lightbars();
                const auto& point = is_upper ? lightbars[id].upper : lightbars[id].lower;
                return Eigen::Vector3d { point.x, point.y, point.z };
            };
            const auto position_plus  = compute(theta + kEpsilon);
            const auto position_minus = compute(theta - kEpsilon);
            if (position_plus.hasNaN()) return Eigen::Vector3d { };
            return ((position_plus - position_minus) / (2.0 * kEpsilon)).eval();
        };

        for (int endpoint = 0; endpoint < 2; ++endpoint) {
            const auto row      = endpoint * 2;
            const auto is_upper = (endpoint == 0);
            const auto& point_ros =
                is_upper ? observable.upper3d[lightbar_id] : observable.lower3d[lightbar_id];

            const auto jacobian_projection_opt = projection_jacobian(point_ros);
            if (!jacobian_projection_opt) continue;

            auto jacobian_position = Eigen::Matrix<double, 3, 11>::Zero().eval();

            jacobian_position(0, kStateX)             = 1.0;
            jacobian_position(1, kStateY)             = 1.0;
            jacobian_position(2, kStateZ)             = 1.0;
            jacobian_position.block<3, 1>(0, kStateA) = theta_jacobian(lightbar_id, is_upper);

            const auto armor_id = int { lightbar_id / 2 };
            const auto direction_forward =
                Eigen::Vector3d { +std::cos(theta), +std::sin(theta), 0.0 };
            const auto direction_right =
                Eigen::Vector3d { -std::sin(theta), +std::cos(theta), 0.0 };

            auto jacobian_forward = jacobian_position.block<3, 1>(0, kStateRF);
            auto jacobian_lateral = jacobian_position.block<3, 1>(0, kStateRL);

            if (armor_id == 0) jacobian_forward = direction_forward;
            if (armor_id == 2) jacobian_forward = -direction_forward;

            if (armor_id == 1) jacobian_lateral = direction_right;
            if (armor_id == 3) jacobian_lateral = -direction_right;

            if (armor_id == 1 || armor_id == 3) jacobian_position(2, kStateHL) = 1.0;

            jacobian.block<2, 11>(row, 0).noalias() =
                *jacobian_projection_opt * r_cam2odom * jacobian_position;
        }

        return jacobian;
    }

public:
    explicit Impl(const Config& cfg) noexcept {
        config = cfg;

        observable.yaw_full_max = cfg.yaw_full_max;
        observable.yaw_part_max = cfg.yaw_part_max;

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

    auto init(std::span<const Armor2d> armors2d) noexcept -> bool {
        constexpr auto kPitch = kPredictedOtherArmorPitch;

        genre = DeviceId::UNKNOWN;
        color = ArmorColor::DARK;

        if (armors2d.empty()) return false;
        genre = armors2d[0].genre;
        color = armors2d[0].color;

        const Armor2d* best_armor2d = nullptr;
        {
            auto best_width = 0.0;
            for (const auto& armor2d : armors2d) {
                const auto w = std::abs(armor2d.tr.x - armor2d.tl.x);
                if (w <= best_width) continue;
                best_width   = w;
                best_armor2d = &armor2d;
            }
            if (!best_armor2d) return false;
        }

        auto pnp          = util::RobustPnpSolution { };
        pnp.input.feature = camera_feature;
        pnp.input.armor2d = *best_armor2d;
        if (!pnp.solve()) return false;

        const auto best_armor3d = pnp.result.armor3d;

        const auto orientation = best_armor3d.orientation.make<Eigen::Quaterniond>();
        const auto translation = best_armor3d.translation.make<Eigen::Vector3d>();

        const auto armor2center = Eigen::Vector3d {
            Eigen::AngleAxisd { -kPitch, orientation * Eigen::Vector3d::UnitY() }
                * (orientation * Eigen::Vector3d::UnitX()),
        };
        auto yaw    = util::normalize_angle(std::atan2(-armor2center.y(), -armor2center.x()));
        auto center = Eigen::Vector3d { translation + 0.2 * armor2center };

        context.posteriors_state           = StateVector::Zero();
        context.posteriors_state[kStateX]  = center.x();
        context.posteriors_state[kStateY]  = center.y();
        context.posteriors_state[kStateZ]  = center.z();
        context.posteriors_state[kStateRF] = 0.2;
        context.posteriors_state[kStateRL] = 0.2;
        context.posteriors_state[kStateHL] = 0.0;

        // 验证 PnP yaw：选可见装甲板数更多的朝向
        observable.update(center, yaw, 0.2, 0.2, 0.0, genre);
        const auto orig_count = observable.visible_armors().size();

        const auto alt_yaw = util::normalize_angle(yaw + std::numbers::pi);
        observable.update(center, alt_yaw, 0.2, 0.2, 0.0, genre);
        const auto alt_count = observable.visible_armors().size();

        if (alt_count > orig_count) yaw = alt_yaw;

        context.posteriors_state[kStateA] = yaw;
        observable.update(center, yaw, 0.2, 0.2, 0.0, genre);

        init_timestamp = Clock::now();
        return true;
    }

    auto predict(double dt) noexcept -> void {
        auto next = context.posteriors_state;
        next[kStateX] += next[kStateVx] * dt;
        next[kStateY] += next[kStateVy] * dt;
        next[kStateZ] += next[kStateVz] * dt;
        next[kStateA] = util::normalize_angle(next[kStateA] + next[kStateW] * dt);

        auto jacobian               = Covariance { Covariance::Identity() };
        jacobian(kStateX, kStateVx) = dt;
        jacobian(kStateY, kStateVy) = dt;
        jacobian(kStateZ, kStateVz) = dt;
        jacobian(kStateA, kStateW)  = dt;

        context.posteriors_state = next;
        context.posteriors_covariance =
            jacobian * context.posteriors_covariance * jacobian.transpose() + context.noise_process;

        observable.update(
            {
                next[kStateX],
                next[kStateY],
                next[kStateZ],
            },
            next[kStateA], next[kStateRF], next[kStateRL], next[kStateHL], genre);
    }

    auto update(const Eigen::Vector2d& upper, const Eigen::Vector2d& lower, int id) noexcept {

        const auto prior_state      = context.posteriors_state;
        const auto prior_covariance = context.posteriors_covariance;

        const auto pred_upper = observable.upper2d[id];
        const auto pred_lower = observable.lower2d[id];

        // innovation (4D)
        auto innovation = Eigen::Vector4d { };
        innovation << upper.x() - pred_upper.x(), upper.y() - pred_upper.y(),
            lower.x() - pred_lower.x(), lower.y() - pred_lower.y();

        // H (4×11): geometry Jacobian
        const auto jacobian = make_observation_jacobian(id);
        if (jacobian.hasNaN()) return;

        // Kalman gain
        const auto innovation_covariance =
            jacobian * prior_covariance * jacobian.transpose() + context.noise_observation;
        const auto innovation_covariance_inv = innovation_covariance.inverse();
        if (innovation_covariance_inv.hasNaN()) return;
        const auto kalman_gain =
            prior_covariance * jacobian.transpose() * innovation_covariance_inv;

        // state update
        auto posterior_state = StateVector { prior_state };
        posterior_state.noalias() += kalman_gain * innovation;

        if (posterior_state.hasNaN() || !posterior_state.allFinite()) return;
        posterior_state[kStateA] = util::normalize_angle(posterior_state[kStateA]);

        posterior_state[kStateRF] = std::clamp(
            posterior_state[kStateRF], config.radius_forward_min, config.radius_forward_max);
        posterior_state[kStateRL] = std::clamp(
            posterior_state[kStateRL], config.radius_lateral_min, config.radius_lateral_max);
        posterior_state[kStateHL] = std::clamp(
            posterior_state[kStateHL], config.height_lateral_min, config.height_lateral_max);

        posterior_state[kStateVx] = std::clamp(posterior_state[kStateVx], -10.0, 10.0);
        posterior_state[kStateVy] = std::clamp(posterior_state[kStateVy], -10.0, 10.0);

        // covariance update
        const auto complement          = Covariance::Identity() - kalman_gain * jacobian;
        auto posterior_covariance      = Covariance { };
        posterior_covariance.noalias() = complement * prior_covariance * complement.transpose();
        posterior_covariance +=
            (kalman_gain * context.noise_observation * kalman_gain.transpose()).eval();
        posterior_covariance = 0.5 * (posterior_covariance + posterior_covariance.transpose());

        context.posteriors_state      = posterior_state;
        context.posteriors_covariance = posterior_covariance;

        // 刷新 observable，下个灯条用最新状态预测
        observable.update(
            {
                posterior_state[kStateX],
                posterior_state[kStateY],
                posterior_state[kStateZ],
            },
            posterior_state[kStateA], posterior_state[kStateRF], posterior_state[kStateRL],
            posterior_state[kStateHL], genre);
    }

    auto full() const {
        auto solution = RobotSolution { };

        const auto state = context.posteriors_state;

        solution.input.center = Eigen::Vector3d { state[kStateX], state[kStateY], state[kStateZ] };
        solution.input.toward =
            Eigen::Quaterniond { Eigen::AngleAxisd { state[kStateA], Eigen::Vector3d::UnitZ() } };
        solution.input.radius_forward = state[kStateRF];
        solution.input.radius_lateral = state[kStateRL];
        solution.input.height_lateral = state[kStateHL];
        solution.input.genre          = genre;
        solution.input.color          = color;

        return solution.solve_armors();
    }

    auto correct(std::span<const Armor2d> armors, std::span<const Lightbar2d> lightbars) noexcept {
        const auto& state = context.posteriors_state;
        observable.update(
            {
                state[kStateX],
                state[kStateY],
                state[kStateZ],
            },
            state[kStateA], state[kStateRF], state[kStateRL], state[kStateHL], genre);

        struct SortedEntry {
            int assigned_id;
            Eigen::Vector2d upper;
            Eigen::Vector2d lower;
        };
        auto sorted = std::vector<SortedEntry> { };

        // Step 1: 交叉匹配，找四角点误差和最小的完整装甲板作为锚
        Armor2d const* anchor_armor = nullptr;

        auto anchor_armor_id = int { -1 };
        auto best_error_sum  = std::numeric_limits<double>::max();

        for (const auto& armor : armors) {
            for (int armor_id = 0; armor_id < 4; ++armor_id) {
                const auto left_id  = armor_id * 2;
                const auto right_id = armor_id * 2 + 1;
                if (!observable.visible[left_id] || !observable.visible[right_id]) continue;

                const auto error_0 = squared_distance(
                    Eigen::Vector2d { armor.tl.x, armor.tl.y }, observable.upper2d[left_id]);
                const auto error_1 = squared_distance(
                    Eigen::Vector2d { armor.bl.x, armor.bl.y }, observable.lower2d[left_id]);
                const auto error_2 = squared_distance(
                    Eigen::Vector2d { armor.tr.x, armor.tr.y }, observable.upper2d[right_id]);
                const auto error_3 = squared_distance(
                    Eigen::Vector2d { armor.br.x, armor.br.y }, observable.lower2d[right_id]);
                const auto sum = error_0 + error_1 + error_2 + error_3;

                if (sum >= best_error_sum) continue;
                best_error_sum  = sum;
                anchor_armor_id = armor_id;
                anchor_armor    = &armor;
            }
        }

        if (!anchor_armor) return;

        // Step 2: 收集全部观测灯条，按 upper.x 排序
        for (const auto& armor : armors) {
            sorted.push_back({ -1, Eigen::Vector2d { armor.tl.x, armor.tl.y },
                Eigen::Vector2d { armor.bl.x, armor.bl.y } });
            sorted.push_back({ -1, Eigen::Vector2d { armor.tr.x, armor.tr.y },
                Eigen::Vector2d { armor.br.x, armor.br.y } });
        }
        for (const auto& lightbar : lightbars) {
            sorted.push_back({
                -1,
                Eigen::Vector2d { lightbar.upper.x, lightbar.upper.y },
                Eigen::Vector2d { lightbar.lower.x, lightbar.lower.y },
            });
        }

        std::ranges::sort(sorted,
            [](const SortedEntry& a, const SortedEntry& b) { return a.upper.x() < b.upper.x(); });

        // Step 3: 锚定位 → 向两侧递推
        {
            auto anchor_left    = std::size_t { 0 };
            auto anchor_right   = std::size_t { 0 };
            auto min_distance_l = std::numeric_limits<double>::max();
            auto min_distance_r = std::numeric_limits<double>::max();
            for (std::size_t i = 0; i < sorted.size(); ++i) {
                const auto distance_l = std::abs(sorted[i].upper.x() - anchor_armor->tl.x);
                const auto distance_r = std::abs(sorted[i].upper.x() - anchor_armor->tr.x);
                if (distance_l < min_distance_l) {
                    min_distance_l = distance_l;
                    anchor_left    = i;
                }
                if (distance_r < min_distance_r) {
                    min_distance_r = distance_r;
                    anchor_right   = i;
                }
            }

            sorted[anchor_left].assigned_id  = anchor_armor_id * 2;
            sorted[anchor_right].assigned_id = anchor_armor_id * 2 + 1;

            for (std::size_t i = anchor_left + 1; i < sorted.size(); ++i)
                sorted[i].assigned_id = (sorted[i - 1].assigned_id + 1) % 8;

            for (std::size_t i = anchor_left; i > 0; --i)
                sorted[i - 1].assigned_id = (sorted[i].assigned_id - 1 + 8) % 8;
        }

        // Step 4: EKF update + 填 tracked
        addition.tracked.clear();
        for (const auto& entry : sorted) {
            update(entry.upper, entry.lower, entry.assigned_id);
            addition.tracked.push_back(
                { entry.assigned_id, Point2d { entry.upper.x(), entry.upper.y() } });
        }

        for (auto&& [index, armor] : std::views::enumerate(addition.armors)) {
            const auto l = static_cast<int>(index) * 2;
            const auto r = l + 1;

            armor.genre = genre;
            armor.color = color;

            const auto& upper = observable.upper2d;
            const auto& lower = observable.lower2d;

            armor.tl = { static_cast<float>(upper[l].x()), static_cast<float>(upper[l].y()) };
            armor.tr = { static_cast<float>(upper[r].x()), static_cast<float>(upper[r].y()) };
            armor.br = { static_cast<float>(lower[r].x()), static_cast<float>(lower[r].y()) };
            armor.bl = { static_cast<float>(lower[l].x()), static_cast<float>(lower[l].y()) };
        }
        update_count += 1;
    }

    auto converge() const -> bool {
        constexpr auto kMinUpdate = std::size_t { 10 };
        constexpr auto kMaxCovXY  = 1.0;

        if (update_count < kMinUpdate) return false;

        const auto& cov = context.posteriors_covariance;
        if (cov(kStateX, kStateX) > kMaxCovXY) return false;
        if (cov(kStateY, kStateY) > kMaxCovXY) return false;

        using namespace std::chrono_literals;
        if (Clock::now() - init_timestamp < 0.1s) return false;

        return true;
    }

    auto diverged() const -> bool {
        const auto& state = context.posteriors_state;
        const auto& cov   = context.posteriors_covariance;
        return std::ranges::any_of(
            std::array {
                cov(kStateX, kStateX) > 150,
                cov(kStateY, kStateY) > 150,

                std::abs(state[kStateX]) > 15.0,
                std::abs(state[kStateY]) > 15.0,
                std::abs(state[kStateZ]) > 02.0,
                std::abs(state[kStateW]) > 10.0 * std::numbers::pi,

                std::abs(state[kStateVx]) > 5.,
                std::abs(state[kStateVy]) > 5.,
                std::abs(state[kStateVz]) > 1.,

                state.hasNaN() == true,
                state.allFinite() == false,
            },
            std::identity { });
    }
};

RobotModel::RobotModel(const Config& cfg) noexcept
    : pimpl { std::make_unique<Impl>(cfg) } { }

RobotModel::~RobotModel() noexcept = default;

auto RobotModel::update_camera(std::array<double, 9> matrix, std::array<double, 5> coeff) noexcept
    -> void {
    pimpl->configure_camera(matrix, coeff);
}

auto RobotModel::init(std::span<const Armor2d> armors) noexcept -> bool {
    return pimpl->init(armors);
}

auto RobotModel::update_transform(const Transform& t) noexcept -> void {
    pimpl->update_transform(t);
}

auto RobotModel::predict(double dt) noexcept -> void { pimpl->predict(dt); }

auto RobotModel::correct(
    std::span<const Armor2d> armors, std::span<const Lightbar2d> lightbars) noexcept -> void {
    pimpl->correct(armors, lightbars);
}

auto RobotModel::state() const noexcept -> State { return pimpl->context.get_state(); }

auto RobotModel::full() const -> std::array<Armor3d, 4> { return pimpl->full(); }

auto RobotModel::converge() const -> bool { return pimpl->converge(); }
auto RobotModel::diverged() const -> bool { return pimpl->diverged(); }

auto RobotModel::addition() const -> const Addition& { return pimpl->addition; }

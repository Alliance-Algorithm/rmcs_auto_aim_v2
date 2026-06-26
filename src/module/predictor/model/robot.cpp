#include "robot.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/reprojection.hpp"
#include "utility/math/robot.hpp"
#include "utility/math/solve_pnp/pnp_solution.hpp"
#include "utility/robot/constant.hpp"

#include <eigen3/Eigen/Geometry>
#include <print>

using namespace rmcs;

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
            diag << 1.0, 1.0, 1.0, // x, y, z
                64.0, 64.0, 64.0, // vx, vy, vz
                0.4, // rotation_angle
                100.0, // rotation_speed
                1e-2, 1e-2, 1e-2; // rf, rl, hl
            posteriors_covariance = diag.asDiagonal();
        }

        auto set_noise(const Config& config) noexcept {
            std::ignore = config;
            noise_process.diagonal() << 1e-4, 1e-4, 1e-4, 5e-3, 5e-3, 5e-3, 5e-4, 1e-2, 1e-8, 1e-8,
                1e-8;
            noise_observation.diagonal() << 2.0, 2.0, 2.0, 2.0;
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
        std::array<Eigen::Vector2d, 8> upper;
        std::array<Eigen::Vector2d, 8> lower;
        std::array<Eigen::Vector3d, 8> upper_3d;
        std::array<Eigen::Vector3d, 8> lower_3d;
        std::array<bool, 8> visible;

        util::CameraFeature cam;
        double yaw_full_max = 60.0 * std::numbers::pi / 180.0;
        double yaw_part_max = 90.0 * std::numbers::pi / 180.0;

        auto update(const Point3d& center, double theta, double rf, double rl, double hl) noexcept
            -> void {

            upper.fill(Eigen::Vector2d::Zero());
            lower.fill(Eigen::Vector2d::Zero());
            upper_3d.fill(Eigen::Vector3d::Zero());
            lower_3d.fill(Eigen::Vector3d::Zero());
            visible.fill(false);

            const auto ros2cv = [](const Point3d& p) -> Point3d {
                return Point3d { -p.y, -p.z, p.x };
            };

            auto solution = RobotSolution { };
            solution.input.center =
                Translation { Eigen::Vector3d { center.x, center.y, center.z } };
            solution.input.toward         = Orientation { Eigen::Quaterniond {
                Eigen::AngleAxisd { theta, Eigen::Vector3d::UnitZ() } } };
            solution.input.radius_forward = rf;
            solution.input.radius_lateral = rl;
            solution.input.height_lateral = hl;

            const auto lightbars = solution.solve_lightbars();
            const auto armors    = solution.solve_armors();

            for (int i = 0; i < 8; ++i) {
                upper_3d[i] = Eigen::Vector3d {
                    lightbars[i].upper.x,
                    lightbars[i].upper.y,
                    lightbars[i].upper.z,
                };
                lower_3d[i] = Eigen::Vector3d {
                    lightbars[i].lower.x,
                    lightbars[i].lower.y,
                    lightbars[i].lower.z,
                };

                const auto up_opt = reproject_point(ros2cv(lightbars[i].upper), cam);
                const auto lo_opt = reproject_point(ros2cv(lightbars[i].lower), cam);
                if (up_opt && lo_opt) {
                    upper[i] = up_opt->make<Eigen::Vector2d>();
                    lower[i] = lo_opt->make<Eigen::Vector2d>();
                } else {
                    upper[i] = Eigen::Vector2d::Zero();
                    lower[i] = Eigen::Vector2d::Zero();
                }
            }

            for (int i = 0; i < 4; ++i) {
                const auto orientation = armors[i].orientation.make<Eigen::Quaterniond>();
                const auto position    = armors[i].translation.make<Eigen::Vector3d>();

                const auto armor_to_center = orientation * Eigen::Vector3d::UnitX();
                const auto armor_face      = -armor_to_center;
                const auto armor_to_cam    = (-position).normalized();
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

    Context context;
    Observable observable;

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
        const auto& s = context.posteriors_state;
        if (s.hasNaN()) {
            return Eigen::Matrix<double, 4, 11>::Zero();
        }
        const auto theta = s[kStateA];
        const auto fx    = config.camera_feature.camera_matrix[0][0];
        const auto fy    = config.camera_feature.camera_matrix[1][1];

        constexpr auto kEpsilon = 1e-4;

        auto jacobian = Eigen::Matrix<double, 4, 11> { };
        jacobian.setZero();

        auto projection_jacobian =
            [fx, fy](const Eigen::Vector3d& ros_pt) -> std::optional<Eigen::Matrix<double, 2, 3>> {
            const auto cv_pt     = Eigen::Vector3d { -ros_pt.y(), -ros_pt.z(), ros_pt.x() };
            const auto Z         = cv_pt.z();
            constexpr auto kMinZ = 0.1;
            if (Z < kMinZ) return std::nullopt;
            // ∂pixel/∂cv
            Eigen::Matrix<double, 2, 3> J_pixel_cv;
            J_pixel_cv << fx / Z, 0, -fx * cv_pt.x() / (Z * Z), 0, fy / Z,
                -fy * cv_pt.y() / (Z * Z);
            // ∂cv/∂ros
            Eigen::Matrix3d J_cv_ros;
            J_cv_ros << 0, -1, 0, 0, 0, -1, 1, 0, 0;
            return J_pixel_cv * J_cv_ros;
        };

        auto theta_jacobian = [&](int id, bool is_upper) {
            auto compute = [&](double t) -> Eigen::Vector3d {
                auto sol = RobotSolution { };
                sol.input.center =
                    Translation { Eigen::Vector3d { s[kStateX], s[kStateY], s[kStateZ] } };
                sol.input.toward         = Orientation { Eigen::Quaterniond {
                    Eigen::AngleAxisd { t, Eigen::Vector3d::UnitZ() } } };
                sol.input.radius_forward = s[kStateRF];
                sol.input.radius_lateral = s[kStateRL];
                sol.input.height_lateral = s[kStateHL];
                const auto bars          = sol.solve_lightbars();
                const auto& pt           = is_upper ? bars[id].upper : bars[id].lower;
                return Eigen::Vector3d { pt.x, pt.y, pt.z };
            };
            const auto p_plus  = compute(theta + kEpsilon);
            const auto p_minus = compute(theta - kEpsilon);
            if (p_plus.hasNaN()) return Eigen::Vector3d { };
            return ((p_plus - p_minus) / (2.0 * kEpsilon)).eval();
        };

        for (int endpoint = 0; endpoint < 2; ++endpoint) {
            const int row       = endpoint * 2;
            const bool is_upper = (endpoint == 0);
            const auto& pt_ros =
                is_upper ? observable.upper_3d[lightbar_id] : observable.lower_3d[lightbar_id];

            const auto J_proj_opt = projection_jacobian(pt_ros);
            if (!J_proj_opt) continue;

            // ∂p3d/∂state (3×11)
            auto J_p3d = Eigen::Matrix<double, 3, 11> { Eigen::Matrix<double, 3, 11>::Zero() };
            J_p3d(0, kStateX)             = 1.0;
            J_p3d(1, kStateY)             = 1.0;
            J_p3d(2, kStateZ)             = 1.0;
            J_p3d.block<3, 1>(0, kStateA) = theta_jacobian(lightbar_id, is_upper);

            const int armor_id = lightbar_id / 2;
            const auto px_dir  = Eigen::Vector3d {
                std::cos(theta),
                std::sin(theta),
                0.0,
            };
            const auto py_dir = Eigen::Vector3d {
                -std::sin(theta),
                std::cos(theta),
                0.0,
            };
            if (armor_id == 0) J_p3d.block<3, 1>(0, kStateRF) = px_dir;
            else if (armor_id == 1) {
                J_p3d.block<3, 1>(0, kStateRL) = py_dir;
                J_p3d(2, kStateHL)             = 1.0;
            } else if (armor_id == 2) J_p3d.block<3, 1>(0, kStateRF) = -px_dir;
            else {
                J_p3d.block<3, 1>(0, kStateRL) = -py_dir;
                J_p3d(2, kStateHL)             = 1.0;
            }

            // chain: J_proj(2×3) * J_p3d(3×11) → 2×11
            jacobian.block<2, 11>(row, 0).noalias() = *J_proj_opt * J_p3d;
        }

        return jacobian;
    }


public:
    explicit Impl(std::span<const Armor2d> armors2d, const Config& cfg) noexcept {
        config                  = cfg;
        observable.cam          = cfg.camera_feature;
        observable.yaw_full_max = cfg.yaw_full_max;
        observable.yaw_part_max = cfg.yaw_part_max;
        genre                   = DeviceId::UNKNOWN;
        color                   = ArmorColor::DARK;

        if (armors2d.empty()) return;
        genre = armors2d[0].genre;
        color = armors2d[0].color;

        auto best_width             = 0.0;
        const Armor2d* best_armor2d = nullptr;
        for (const auto& armor2d : armors2d) {
            const auto w = std::abs(armor2d.tr.x - armor2d.tl.x);
            if (w <= best_width) continue;
            best_width   = w;
            best_armor2d = &armor2d;
        }
        if (!best_armor2d) return;

        auto pnp                      = util::RobustPnpSolution { };
        pnp.input.feature             = cfg.camera_feature;
        pnp.input.feature.orientation = Orientation::kIdentity();
        pnp.input.feature.translation = Translation::kZero();
        pnp.input.armor2d             = *best_armor2d;
        if (!pnp.solve()) return;

        const auto best_armor3d = pnp.result.armor3d;

        constexpr auto pitch = kPredictedOtherArmorPitch;
        const auto q         = best_armor3d.orientation.make<Eigen::Quaterniond>();
        const auto t         = best_armor3d.translation.make<Eigen::Vector3d>();
        const auto armor_to_center =
            Eigen::Vector3d { Eigen::AngleAxisd { -pitch, q * Eigen::Vector3d::UnitY() }
                * (q * Eigen::Vector3d::UnitX()) };
        const auto obs_yaw =
            util::normalize_angle(std::atan2(-armor_to_center.y(), -armor_to_center.x()));
        auto center    = Eigen::Vector3d { t + 0.2 * armor_to_center };
        auto yaw       = obs_yaw;
        auto center_pt = Point3d { center.x(), center.y(), center.z() };

        context.posteriors_state           = StateVector::Zero();
        context.posteriors_state[kStateX]  = center.x();
        context.posteriors_state[kStateY]  = center.y();
        context.posteriors_state[kStateZ]  = center.z();
        context.posteriors_state[kStateRF] = 0.2;
        context.posteriors_state[kStateRL] = 0.2;
        context.posteriors_state[kStateHL] = 0.0;

        context.set_noise(config);
        context.reset_covariance();

        // 验证 PnP yaw：选可见装甲板数更多的朝向
        observable.update(center_pt, yaw, 0.2, 0.2, 0.0);
        const auto orig_count = observable.visible_armors().size();

        const auto alt_yaw = util::normalize_angle(yaw + std::numbers::pi);
        observable.update(center_pt, alt_yaw, 0.2, 0.2, 0.0);
        const auto alt_count = observable.visible_armors().size();

        if (alt_count > orig_count) yaw = alt_yaw;

        context.posteriors_state[kStateA] = yaw;
        observable.update(center_pt, yaw, 0.2, 0.2, 0.0);
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

        observable.update(Point3d { next[kStateX], next[kStateY], next[kStateZ] }, next[kStateA],
            next[kStateRF], next[kStateRL], next[kStateHL]);
    }

    auto update(
        const Eigen::Vector2d& upper, const Eigen::Vector2d& lower, int lightbar_id) noexcept {
        const auto prior_state      = context.posteriors_state;
        const auto prior_covariance = context.posteriors_covariance;

        const auto pred_upper = observable.upper[lightbar_id];
        const auto pred_lower = observable.lower[lightbar_id];

        // innovation (4D)
        Eigen::Vector4d innovation;
        innovation << upper.x() - pred_upper.x(), upper.y() - pred_upper.y(),
            lower.x() - pred_lower.x(), lower.y() - pred_lower.y();
        // H (4×11): geometry Jacobian
        const auto jacobian = make_observation_jacobian(lightbar_id);
        if (jacobian.hasNaN()) {
            return;
        }

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
        posterior_state[kStateA] = util::normalize_angle(posterior_state[kStateA]);

        posterior_state[kStateRF] = std::clamp(
            posterior_state[kStateRF], config.radius_forward_min, config.radius_forward_max);
        posterior_state[kStateRL] = std::clamp(
            posterior_state[kStateRL], config.radius_lateral_min, config.radius_lateral_max);
        posterior_state[kStateHL] = std::clamp(
            posterior_state[kStateHL], config.height_lateral_min, config.height_lateral_max);

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
        observable.update(Point3d { posterior_state[kStateX], posterior_state[kStateY],
                              posterior_state[kStateZ] },
            posterior_state[kStateA], posterior_state[kStateRF], posterior_state[kStateRL],
            posterior_state[kStateHL]);
    }

    auto full() const -> std::array<Armor3d, 4> {
        const auto s = context.posteriors_state;

        auto solution = RobotSolution { };
        solution.input.center =
            Translation { Eigen::Vector3d { s[kStateX], s[kStateY], s[kStateZ] } };
        solution.input.toward         = Orientation { Eigen::Quaterniond {
            Eigen::AngleAxisd { s[kStateA], Eigen::Vector3d::UnitZ() } } };
        solution.input.radius_forward = s[kStateRF];
        solution.input.radius_lateral = s[kStateRL];
        solution.input.height_lateral = s[kStateHL];
        solution.input.genre          = genre;
        solution.input.color          = color;

        return solution.solve_armors();
    }

    auto correct(std::span<const Armor2d> armors, std::span<const Lightbar2d> lightbars) noexcept {
        observable.update(
            Point3d {
                context.posteriors_state[kStateX],
                context.posteriors_state[kStateY],
                context.posteriors_state[kStateZ],
            },
            context.posteriors_state[kStateA], context.posteriors_state[kStateRF],
            context.posteriors_state[kStateRL], context.posteriors_state[kStateHL]);

        struct SortedEntry {
            int assigned_id;
            Eigen::Vector2d upper;
            Eigen::Vector2d lower;
        };
        auto sorted = std::vector<SortedEntry> { };

        // Step 1: 交叉匹配，找四角点误差和最小的完整装甲板作为锚
        auto anchor_armor_id        = int { -1 };
        auto best_error_sum         = std::numeric_limits<double>::max();
        const Armor2d* anchor_armor = nullptr;

        for (const auto& armor : armors) {
            for (int armor_id = 0; armor_id < 4; ++armor_id) {
                const auto left_id  = armor_id * 2;
                const auto right_id = armor_id * 2 + 1;
                if (!observable.visible[left_id] || !observable.visible[right_id]) continue;

                const auto error_0 = squared_distance(
                    Eigen::Vector2d { armor.tl.x, armor.tl.y }, observable.upper[left_id]);
                const auto error_1 = squared_distance(
                    Eigen::Vector2d { armor.bl.x, armor.bl.y }, observable.lower[left_id]);
                const auto error_2 = squared_distance(
                    Eigen::Vector2d { armor.tr.x, armor.tr.y }, observable.upper[right_id]);
                const auto error_3 = squared_distance(
                    Eigen::Vector2d { armor.br.x, armor.br.y }, observable.lower[right_id]);
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
                Eigen::Vector2d {
                    static_cast<double>(lightbar.upper.x), static_cast<double>(lightbar.upper.y) },
                Eigen::Vector2d {
                    static_cast<double>(lightbar.lower.x), static_cast<double>(lightbar.lower.y) },
            });
        }

        std::ranges::sort(sorted,
            [](const SortedEntry& a, const SortedEntry& b) { return a.upper.x() < b.upper.x(); });

        // Step 3: 锚定位 → 向两侧递推
        {
            auto anchor_left  = std::size_t { 0 };
            auto anchor_right = std::size_t { 0 };
            auto min_dist_l   = std::numeric_limits<double>::max();
            auto min_dist_r   = std::numeric_limits<double>::max();
            for (std::size_t i = 0; i < sorted.size(); ++i) {
                const auto dx_l = std::abs(sorted[i].upper.x() - anchor_armor->tl.x);
                const auto dx_r = std::abs(sorted[i].upper.x() - anchor_armor->tr.x);
                if (dx_l < min_dist_l) {
                    min_dist_l  = dx_l;
                    anchor_left = i;
                }
                if (dx_r < min_dist_r) {
                    min_dist_r   = dx_r;
                    anchor_right = i;
                }
            }

            sorted[anchor_left].assigned_id  = anchor_armor_id * 2;
            sorted[anchor_right].assigned_id = anchor_armor_id * 2 + 1;

            for (std::size_t i = anchor_left + 1; i < sorted.size(); ++i)
                sorted[i].assigned_id = (sorted[i - 1].assigned_id + 1) % 8;

            for (std::size_t i = anchor_left; i > 0; --i)
                sorted[i - 1].assigned_id = (sorted[i].assigned_id - 1 + 8) % 8;
        }

        // Step 4: EKF update + println + 填 tracked
        addition.tracked.clear();
        std::print("[match] ");
        for (const auto& entry : sorted) {
            update(entry.upper, entry.lower, entry.assigned_id);
            std::print("{}@{:.0f} ", entry.assigned_id, entry.upper.x());
            addition.tracked.push_back(
                { entry.assigned_id, Point2d { entry.upper.x(), entry.upper.y() } });
        }
    }
};

RobotModel::RobotModel(std::span<const Armor2d> armors, const Config& cfg) noexcept
    : pimpl { std::make_unique<Impl>(armors, cfg) } { }

RobotModel::~RobotModel() noexcept = default;

auto RobotModel::configure(const Config& cfg) noexcept -> void { pimpl->config = cfg; }

auto RobotModel::predict(double dt) noexcept -> void { pimpl->predict(dt); }

auto RobotModel::correct(std::span<const Armor2d> a, std::span<const Lightbar2d> l) noexcept
    -> void {
    pimpl->correct(a, l);
}

auto RobotModel::state() noexcept -> State { return pimpl->context.get_state(); }

auto RobotModel::full() const -> std::array<Armor3d, 4> { return pimpl->full(); }

auto RobotModel::addition() const -> const Addition& { return pimpl->addition; }

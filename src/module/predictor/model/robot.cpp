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
            noise_process.diagonal() << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2, 1e-3, 1e-2, 1e-6, 1e-6,
                1e-6;
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
        std::array<cv::Point2f, 8> upper;
        std::array<cv::Point2f, 8> lower;
        std::array<bool, 8> visible;

        auto update(const Point3d& center, double theta, const util::CameraFeature& camera_feature,
            double yaw_full_max = 60.0 * std::numbers::pi / 180.0,
            double yaw_part_max = 90.0 * std::numbers::pi / 180.0) noexcept -> void {

            const auto ros2cv = [](const Point3d& p) -> Point3d {
                return Point3d { -p.y, -p.z, p.x };
            };

            auto solution = RobotSolution { };
            solution.input.center =
                Translation { Eigen::Vector3d { center.x, center.y, center.z } };
            solution.input.toward         = Orientation { Eigen::Quaterniond {
                Eigen::AngleAxisd { theta, Eigen::Vector3d::UnitZ() } } };
            solution.input.radius_forward = 0.2;
            solution.input.radius_lateral = 0.2;
            solution.input.height_lateral = 0.0;

            const auto lightbars = solution.solve_lightbars();
            const auto armors    = solution.solve_armors();

            for (int i = 0; i < 8; ++i) {
                const auto up_opt = reproject_point(ros2cv(lightbars[i].upper), camera_feature);
                const auto lo_opt = reproject_point(ros2cv(lightbars[i].lower), camera_feature);
                if (up_opt && lo_opt) {
                    upper[i] = up_opt->make<cv::Point2f>();
                    lower[i] = lo_opt->make<cv::Point2f>();
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

        auto visible_lightbars() const noexcept -> std::vector<int> {
            auto result = std::vector<int> { };
            for (int i = 0; i < 8; ++i)
                if (visible[i]) result.push_back(i);
            return result;
        }

        auto visible_armors() const noexcept -> std::vector<int> {
            auto result = std::vector<int> { };
            for (int i = 0; i < 4; ++i)
                if (visible[i * 2] && visible[i * 2 + 1]) result.push_back(i);
            return result;
        }
    };

    Context context;
    Config config { };
    ArmorGenre genre { DeviceId::UNKNOWN };
    ArmorColor color { ArmorColor::DARK };
    Addition addition { };
    Observable observable;

private:
    static auto squared_distance(const cv::Point2f& a, const cv::Point2f& b) noexcept -> float {
        const auto dx = a.x - b.x;
        const auto dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    auto update_addition() noexcept -> void {
        addition.upper = observable.upper;
        addition.lower = observable.lower;
    }

public:
    explicit Impl(std::span<const Armor2d> armors2d, const Config& cfg) noexcept {
        config = cfg;
        genre  = DeviceId::UNKNOWN;
        color  = ArmorColor::DARK;

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
        const auto center = Eigen::Vector3d { t + 0.2 * armor_to_center };

        context.posteriors_state           = StateVector::Zero();
        context.posteriors_state[kStateX]  = center.x();
        context.posteriors_state[kStateY]  = center.y();
        context.posteriors_state[kStateZ]  = center.z();
        context.posteriors_state[kStateA]  = obs_yaw;
        context.posteriors_state[kStateRF] = 0.2;
        context.posteriors_state[kStateRL] = 0.2;
        context.posteriors_state[kStateHL] = 0.0;

        context.set_noise(config);
        context.reset_covariance();
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
            config.camera_feature, config.yaw_full_max, config.yaw_part_max);
        update_addition();
    }

    auto set_state(const Point3d& c, double ta, double rf = 0.2, double rl = 0.2,
        double hl = 0.0) noexcept -> void {
        context.posteriors_state[kStateX]  = c.x;
        context.posteriors_state[kStateY]  = c.y;
        context.posteriors_state[kStateZ]  = c.z;
        context.posteriors_state[kStateA]  = ta;
        context.posteriors_state[kStateRF] = rf;
        context.posteriors_state[kStateRL] = rl;
        context.posteriors_state[kStateHL] = hl;

        observable.update(Point3d { c.x, c.y, c.z }, ta, config.camera_feature, config.yaw_full_max,
            config.yaw_part_max);
        update_addition();
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

    auto correct(std::span<const Armor2d> armors, std::span<const Lightbar2d> lightbars) noexcept
        -> void {

        observable.update(
            Point3d {
                context.posteriors_state[kStateX],
                context.posteriors_state[kStateY],
                context.posteriors_state[kStateZ],
            },
            context.posteriors_state[kStateA], config.camera_feature, config.yaw_full_max,
            config.yaw_part_max);

        struct SortedEntry {
            float x;
            int assigned_id;
            cv::Point2f point;
        };
        auto sorted = std::vector<SortedEntry> { };

        // Step 1: 交叉匹配，找四角点误差和最小的完整装甲板作为锚
        auto anchor_armor_id        = int { -1 };
        auto best_error_sum         = std::numeric_limits<float>::max();
        const Armor2d* anchor_armor = nullptr;

        for (const auto& armor : armors) {
            for (int armor_id = 0; armor_id < 4; ++armor_id) {
                const auto left_id  = armor_id * 2;
                const auto right_id = armor_id * 2 + 1;
                if (!observable.visible[left_id] || !observable.visible[right_id]) continue;

                const auto error_0 = squared_distance(armor.tl, observable.upper[left_id]);
                const auto error_1 = squared_distance(armor.bl, observable.lower[left_id]);
                const auto error_2 = squared_distance(armor.tr, observable.upper[right_id]);
                const auto error_3 = squared_distance(armor.br, observable.lower[right_id]);
                const auto sum     = error_0 + error_1 + error_2 + error_3;

                if (sum >= best_error_sum) continue;
                best_error_sum  = sum;
                anchor_armor_id = armor_id;
                anchor_armor    = &armor;
            }
        }

        if (!anchor_armor) return;

        // Step 2: 收集全部观测灯条，按 upper.x 排序
        for (const auto& armor : armors) {
            sorted.push_back({ armor.tl.x, -1,
                cv::Point2f { static_cast<float>((armor.tl.x + armor.bl.x) * 0.5f),
                    static_cast<float>((armor.tl.y + armor.bl.y) * 0.5f) } });
            sorted.push_back({ armor.tr.x, -1,
                cv::Point2f { static_cast<float>((armor.tr.x + armor.br.x) * 0.5f),
                    static_cast<float>((armor.tr.y + armor.br.y) * 0.5f) } });
        }
        for (const auto& lightbar : lightbars) {
            sorted.push_back({ static_cast<float>(lightbar.upper.x), -1,
                cv::Point2f {
                    static_cast<float>((lightbar.upper.x + lightbar.lower.x) * 0.5f),
                    static_cast<float>((lightbar.upper.y + lightbar.lower.y) * 0.5f),
                } });
        }

        std::ranges::sort(
            sorted, [](const SortedEntry& a, const SortedEntry& b) { return a.x < b.x; });

        // Step 3: 锚定位 → 向两侧递推
        {
            auto anchor_left  = std::size_t { 0 };
            auto anchor_right = std::size_t { 0 };
            auto min_dist_l   = std::numeric_limits<float>::max();
            auto min_dist_r   = std::numeric_limits<float>::max();
            for (std::size_t i = 0; i < sorted.size(); ++i) {
                const auto dx_l = std::abs(sorted[i].x - anchor_armor->tl.x);
                const auto dx_r = std::abs(sorted[i].x - anchor_armor->tr.x);
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

        // Step 4: println + 填 tracked
        addition.tracked.clear();
        std::print("[match] ");
        for (const auto& entry : sorted) {
            std::print("{}@{:.0f} ", entry.assigned_id, static_cast<double>(entry.x));
            addition.tracked.push_back({ entry.assigned_id, entry.point });
        }
    }
};

RobotModel::RobotModel(std::span<const Armor2d> armors, const Config& cfg) noexcept
    : pimpl { std::make_unique<Impl>(armors, cfg) } { }

RobotModel::~RobotModel() noexcept = default;

auto RobotModel::configure(const Config& cfg) noexcept -> void { pimpl->config = cfg; }

auto RobotModel::set_state(
    const Point3d& center, double theta, double rf, double rl, double hl) noexcept -> void {
    pimpl->set_state(center, theta, rf, rl, hl);
}

auto RobotModel::predict(double dt) noexcept -> void { pimpl->predict(dt); }

auto RobotModel::correct(std::span<const Armor2d> a, std::span<const Lightbar2d> l) noexcept
    -> void {
    pimpl->correct(a, l);
}

auto RobotModel::state() noexcept -> State { return pimpl->context.get_state(); }

auto RobotModel::full() const -> std::array<Armor3d, 4> { return pimpl->full(); }

auto RobotModel::current() const -> Armor3d { return { }; }

auto RobotModel::addition() const -> const Addition& { return pimpl->addition; }

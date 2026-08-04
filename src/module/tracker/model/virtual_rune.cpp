#include "virtual_rune.hpp"

#include "module/tracker/model/rune.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/camera.hpp"
#include "utility/math/conversion.hpp"
#include "utility/math/reprojection.hpp"

#include <eigen3/Eigen/Geometry>

#include <cmath>
#include <numbers>
#include <optional>
#include <ranges>

using namespace rmcs;
using namespace rmcs::util;

struct VirtualRuneModel::Impl {
    static constexpr auto kSmallRuneSpeed     = std::numbers::pi / 3.0; // 小符恒速（规则）
    static constexpr auto kLargeRuneFrequency = 1.884; // 大符正弦频率（规则）
    static constexpr auto kLargeRuneAmplitude = 1.0; // 大符正弦幅度（规则中值）
    static constexpr auto kBladeSwitchSeconds = 100.0; // 激活符叶轮转周期
    static constexpr auto kMinZ               = 0.1;

    Config config { };
    CameraFeature camera;
    RuneModel::State state { };
    Timestamp last_stamp { };
    double blade_elapsed = 0.0;

    std::vector<RuneIcon> icons_;
    std::vector<RuneBullseye> bullseyes_;

    explicit Impl(const Config& cfg) noexcept
        : config { cfg } {

        state.x              = config.x;
        state.y              = config.y;
        state.z              = config.z;
        state.face_yaw       = config.face_yaw;
        state.rotation_angle = 0.0;
        state.rotation_speed = 0.0;
        state.sine_valid     = config.large;
        state.sine_t         = 0.0;
        state.sine_phase     = 0.0;

        if (config.large) {
            // 规则正弦：spd = a·sin(1.884t) + 2.0 − a
            state.sine_a     = kLargeRuneAmplitude;
            state.sine_v     = 2.0 - kLargeRuneAmplitude;
            state.sine_omega = kLargeRuneFrequency;
        } else {
            state.rotation_speed = kSmallRuneSpeed;
        }

        camera.from(config.camera_matrix);
        camera.from(config.distort_coeff);
    }

    auto update_camera(const std::array<double, 9>& matrix) noexcept -> void {
        config.camera_matrix = matrix;
        camera.from(matrix);
    }

    auto update_camera(const std::array<double, 5>& coeff) noexcept -> void {
        config.distort_coeff = coeff;
        camera.from(coeff);
    }

    auto update_transform(const Transform& t) noexcept -> void {
        camera.translation = t.translation;
        camera.orientation = t.orientation;
    }

    auto update(Timestamp timestamp) noexcept -> void {
        if (last_stamp != Timestamp { }) {
            const auto dt = std::chrono::duration<double> { timestamp - last_stamp }.count();
            state.transition(dt);
            blade_elapsed += dt;
        }
        last_stamp = timestamp;

        const auto active_blade =
            static_cast<std::size_t>(std::floor(blade_elapsed / kBladeSwitchSeconds)) % 5;

        const auto q_odom_from_cam = camera.orientation.make<Eigen::Quaterniond>();
        const auto q_cam_from_odom = q_odom_from_cam.conjugate();
        const auto cam_position    = camera.translation.make<Eigen::Vector3d>();

        const auto project = [&](const Eigen::Vector3d& world) -> std::optional<Point2d> {
            const auto point_cam = q_cam_from_odom * (world - cam_position);
            const auto point_ocv = util::ros2opencv_position(point_cam);
            if (point_ocv.z() <= kMinZ) return std::nullopt;
            return util::reproject_point(Point3d { point_ocv }, camera);
        };

        const auto r_face = Eigen::AngleAxisd { state.face_yaw, Eigen::Vector3d::UnitZ() };
        const auto center = Eigen::Vector3d { state.x, state.y, state.z };

        // blade 局部系 → 绕 x 旋转 θ 就位 → r_face 到世界（kPoints 同源变换）
        const auto to_world = [&](double theta, const Point3d& local) {
            const auto cos_t = std::cos(theta);
            const auto sin_t = std::sin(theta);
            return Eigen::Vector3d {
                center
                + r_face * Eigen::Vector3d {
                               local.x,
                               local.y * cos_t - local.z * sin_t,
                               local.y * sin_t + local.z * cos_t,
                           },
            };
        };

        icons_.clear();
        bullseyes_.clear();

        // R 标：kPoints[0] = kIcon（y=z=0，旋转恒等，θ=0 即可）
        const auto icon_world = to_world(0.0, RunePagePoints::kPoints[0]);
        if (const auto pixel = project(icon_world)) {
            icons_.push_back({ *pixel, 1.0 });
        }

        // 仅生成待击打符叶：小符 1 个，大符 2 个（间隔 2 位）
        auto inactive_blades = std::vector<std::size_t> { active_blade };
        if (config.large) inactive_blades.push_back((active_blade + 2) % 5);

        // 符叶：kPoints 不含中心，中心取 kT/kB 均值，角点即 kPoints[1..4]
        for (const auto blade : inactive_blades) {
            const auto theta =
                state.rotation_angle + util::deg2rad(72.0 * static_cast<double>(blade));

            const auto blade_center = to_world(theta,
                Point3d {
                    (RunePagePoints::kT.make<Eigen::Vector3d>()
                        + RunePagePoints::kB.make<Eigen::Vector3d>())
                        * 0.5,
                });
            const auto center_pixel = project(blade_center);
            if (!center_pixel) continue;

            auto bullseye   = RuneBullseye { };
            bullseye.center = *center_pixel;
            bullseye.active = false;
            bullseye.score  = 1.0;

            const auto corner_worlds = std::array {
                to_world(theta, RunePagePoints::kPoints[1]),
                to_world(theta, RunePagePoints::kPoints[2]),
                to_world(theta, RunePagePoints::kPoints[3]),
                to_world(theta, RunePagePoints::kPoints[4]),
            };

            for (const auto& [corner_index, corner_world] : corner_worlds | std::views::enumerate) {
                bullseye.corners[corner_index] = project(corner_world).value_or(bullseye.center);
            }

            bullseyes_.push_back(bullseye);
        }
    }

    auto icons() const noexcept -> std::span<const RuneIcon> { return icons_; }

    auto bullseyes() const noexcept -> std::span<const RuneBullseye> { return bullseyes_; }
};

VirtualRuneModel::VirtualRuneModel(const Config& config) noexcept
    : pimpl { std::make_unique<Impl>(config) } { }

VirtualRuneModel::~VirtualRuneModel() noexcept = default;

auto VirtualRuneModel::update_camera(const std::array<double, 9>& matrix) noexcept -> void {
    pimpl->update_camera(matrix);
}

auto VirtualRuneModel::update_camera(const std::array<double, 5>& coeff) noexcept -> void {
    pimpl->update_camera(coeff);
}

auto VirtualRuneModel::update_transform(const Transform& t) noexcept -> void {
    pimpl->update_transform(t);
}

auto VirtualRuneModel::update(Timestamp timestamp) noexcept -> void { pimpl->update(timestamp); }

auto VirtualRuneModel::icons() const noexcept -> std::span<const RuneIcon> {
    return pimpl->icons();
}

auto VirtualRuneModel::bullseyes() const noexcept -> std::span<const RuneBullseye> {
    return pimpl->bullseyes();
}

#include "module/tracker/model/rune.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/camera.hpp"
#include "utility/math/reprojection.hpp"
#include "utility/robot/rune.hpp"

#include <algorithm>
#include <array>
#include <numbers>
#include <print>
#include <random>
#include <span>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace rmcs;
using namespace rmcs::util;

namespace {

constexpr auto kBladeCount = std::size_t { 5 };
constexpr auto kBladeAngle = 2.0 * std::numbers::pi / 5.0;
constexpr auto kRadius     = kRuneGlobalRadius;

// ===== 真值运动 =====
struct MockRune {
    // 符盘中心与朝向（Odom），固定。中心与相机等高（相机在原点平视），
    // 避免符盘投影跑到画面外
    inline static const auto kCenter = Eigen::Vector3d { 5.0, 0.0, 0.0 };
    static constexpr auto kPlaneYaw  = 0.35; // 法向大致指向操作手，略偏斜（真实极少完美正对）

    static constexpr auto kSmallSpeed   = std::numbers::pi / 3.0;
    static constexpr auto kSwitchPeriod = 2.5; // 命中节奏（秒）

    RuneModel::Kind kind;
    int lit_count;

    // 大符正弦真值（每次激活随机重置）
    double sin_a     = 0.0;
    double sin_omega = 0.0;
    double sin_t0    = 0.0;

    double angle      = 0.0; // 累积相位
    double last_switch = 0.0;
    std::array<bool, 5> lit = { };

    std::mt19937& engine;

    MockRune(RuneModel::Kind k, std::mt19937& eng) noexcept
        : kind { k }
        , lit_count { k == RuneModel::Kind::Big ? 2 : 1 }
        , engine { eng } {
        randomize_sinusoid(0.0);
        relight();
    }

    auto randomize_sinusoid(double now) -> void {
        auto a_dist = std::uniform_real_distribution<double> { 0.780, 1.045 };
        auto w_dist = std::uniform_real_distribution<double> { 1.884, 2.000 };
        sin_a       = a_dist(engine);
        sin_omega   = w_dist(engine);
        sin_t0      = now;
    }

    auto relight() -> void {
        lit.fill(false);
        auto idx = std::vector<std::size_t> (kBladeCount);
        std::iota(idx.begin(), idx.end(), 0);
        std::ranges::shuffle(idx, engine);
        for (int i = 0; i < lit_count; ++i) {
            lit[idx[static_cast<std::size_t>(i)]] = true;
        }
    }

    auto omega(double now) const -> double {
        if (kind == RuneModel::Kind::Small) return kSmallSpeed;
        const auto t = now - sin_t0;
        return sin_a * std::sin(sin_omega * t) + (2.090 - sin_a);
    }

    auto step(double dt, double now) -> void {
        angle = util::normalize_angle(angle + omega(now) * dt);
        if (now - last_switch >= kSwitchPeriod) {
            last_switch = now;
            if (kind == RuneModel::Kind::Big) randomize_sinusoid(now);
            relight();
        }
    }

    // 第 i 个扇叶靶心中心（Odom），与 RuneModel::get_aimpoints 约定一致
    auto blade_center(std::size_t i) const -> Eigen::Vector3d {
        const auto phase   = angle + static_cast<double>(i) * kBladeAngle;
        const auto local_y = -kRadius * std::sin(phase);
        const auto local_z = +kRadius * std::cos(phase);

        const auto cos_yaw = std::cos(kPlaneYaw);
        const auto sin_yaw = std::sin(kPlaneYaw);
        return Eigen::Vector3d {
            kCenter.x() - sin_yaw * local_y,
            kCenter.y() + cos_yaw * local_y,
            kCenter.z() + local_z,
        };
    }

    // 第 i 个扇叶十字四端点（Odom）：kT/kL/kB/kR，与 make_bullseye 约定一致
    auto blade_corners(std::size_t i) const -> std::array<Eigen::Vector3d, 4> {
        const auto phase    = angle + static_cast<double>(i) * kBladeAngle;
        const auto rotation = Eigen::AngleAxisd { kPlaneYaw, Eigen::Vector3d::UnitZ() }
            * Eigen::AngleAxisd { phase, Eigen::Vector3d::UnitX() };
        const auto to_odom = [&](const Point3d& local) {
            const Eigen::Vector3d v = rotation * Eigen::Vector3d { local.x, local.y, local.z };
            return Eigen::Vector3d { kCenter + v };
        };
        return {
            to_odom(RunePagePoints::kT),
            to_odom(RunePagePoints::kL),
            to_odom(RunePagePoints::kB),
            to_odom(RunePagePoints::kR),
        };
    }
};

// ===== 观测投影 =====
struct Projector {
    // 与真实相机内参一致（config.yaml），1440x1080
    static constexpr auto kFx = 1722.23;
    static constexpr auto kFy = 1724.88;
    static constexpr auto kCx = 701.31;
    static constexpr auto kCy = 564.58;

    CameraFeature camera;
    std::mt19937& engine;
    std::normal_distribution<double>& pixel_noise;

    Projector(std::mt19937& eng, std::normal_distribution<double>& noise) noexcept
        : engine { eng }
        , pixel_noise { noise } {
        camera.camera_matrix = { std::array { kFx, 0.0, kCx }, std::array { 0.0, kFy, kCy },
            std::array { 0.0, 0.0, 1.0 } };
        camera.distort_coeff = { 0.0, 0.0, 0.0, 0.0, 0.0 };
        camera.orientation   = Orientation::kIdentity();
        camera.translation   = Translation::kZero();
    }

    // Odom(ROS: x前 y左 z上) → OpenCV 相机系(x右 y下 z前) → 像素
    auto project(const Eigen::Vector3d& odom) -> std::optional<Point2d> {
        return reproject_point(Point3d { -odom.y(), -odom.z(), odom.x() }, camera);
    }
    auto noisy(const Point2d& p) -> Point2d {
        return Point2d { p.x + pixel_noise(engine), p.y + pixel_noise(engine) };
    }

    // 生成第 i 扇叶的靶心观测（十字四点 + R 标）；角点顺序故意打乱
    auto make_bullseye(const MockRune& rune, std::size_t i, bool active)
        -> std::optional<RuneBullseye> {
        const auto phase = rune.angle + static_cast<double>(i) * kBladeAngle;

        // 符盘局部 → Odom：R_z(plane_yaw)·R_x(phase)
        const auto rotation =
            Eigen::AngleAxisd { MockRune::kPlaneYaw, Eigen::Vector3d::UnitZ() }
            * Eigen::AngleAxisd { phase, Eigen::Vector3d::UnitX() };

        const auto to_odom = [&](const Point3d& local) {
            const Eigen::Vector3d v = rotation * Eigen::Vector3d { local.x, local.y, local.z };
            return Eigen::Vector3d { MockRune::kCenter + v };
        };

        const auto center_2d = project(to_odom(RunePagePoints::kB)); // 靶心取十字底端附近
        const auto t_2d       = project(to_odom(RunePagePoints::kT));
        const auto l_2d       = project(to_odom(RunePagePoints::kL));
        const auto b_2d       = project(to_odom(RunePagePoints::kB));
        const auto r_2d       = project(to_odom(RunePagePoints::kR));
        if (!center_2d || !t_2d || !l_2d || !b_2d || !r_2d) return std::nullopt;

        // 靶心中心用四点均值
        auto center = RuneBullseye { };
        center.center = noisy(Point2d {
            (t_2d->x + l_2d->x + b_2d->x + r_2d->x) / 4.0,
            (t_2d->y + l_2d->y + b_2d->y + r_2d->y) / 4.0,
        });
        center.active = active;
        center.score  = 1.0;

        auto corners = std::array<Point2d, 4> {
            noisy(*t_2d), noisy(*l_2d), noisy(*b_2d), noisy(*r_2d),
        };
        // 故意打乱角点顺序，考验 Model 的方向枚举
        // std::ranges::shuffle(corners, engine);
        center.corners = corners;

        return center;
    }

    auto make_icon(const MockRune& rune) -> std::optional<RuneIcon> {
        const auto rotation =
            Eigen::AngleAxisd { MockRune::kPlaneYaw, Eigen::Vector3d::UnitZ() }.toRotationMatrix();
        const Eigen::Vector3d v = rotation
            * Eigen::Vector3d { RunePagePoints::kIcon.x, RunePagePoints::kIcon.y,
                RunePagePoints::kIcon.z };
        const Eigen::Vector3d icon_3d = MockRune::kCenter + v;
        const auto icon_2d            = project(icon_3d);
        if (!icon_2d) return std::nullopt;

        auto icon   = RuneIcon { };
        icon.center = noisy(*icon_2d);
        icon.score  = 1.0;
        return icon;
    }
};

// ===== 3D 可视化 =====
struct MarkerHelper {
    visualization_msgs::msg::MarkerArray array { };
    int index          = 0;
    rclcpp::Time stamp = { 0, 0, RCL_ROS_TIME };

    auto make(const std::string& ns) -> visualization_msgs::msg::Marker& {
        auto& m           = array.markers.emplace_back();
        m.header.frame_id = "odom_imu_link";
        m.header.stamp    = stamp;
        m.ns              = ns;
        m.id              = index++;
        m.action          = visualization_msgs::msg::Marker::ADD;
        m.lifetime        = rclcpp::Duration::from_seconds(0.2);
        return m;
    }

    auto add_sphere(const std::string& ns, const Eigen::Vector3d& pos, double r, double cr,
        double cg, double cb, double ca) -> void {
        auto& m   = make(ns);
        m.type    = visualization_msgs::msg::Marker::SPHERE;
        m.scale.x = m.scale.y = m.scale.z = static_cast<float>(r);
        m.color.r = static_cast<float>(cr);
        m.color.g = static_cast<float>(cg);
        m.color.b = static_cast<float>(cb);
        m.color.a = static_cast<float>(ca);
        m.pose.position.x    = pos.x();
        m.pose.position.y    = pos.y();
        m.pose.position.z    = pos.z();
        m.pose.orientation.w = 1.0;
    }

    auto add_points(const std::string& ns, std::span<const Eigen::Vector3d> pts, double size,
        double cr, double cg, double cb, double ca) -> void {
        auto& m   = make(ns);
        m.type    = visualization_msgs::msg::Marker::POINTS;
        m.scale.x = m.scale.y = static_cast<float>(size);
        m.color.r = static_cast<float>(cr);
        m.color.g = static_cast<float>(cg);
        m.color.b = static_cast<float>(cb);
        m.color.a = static_cast<float>(ca);
        m.pose.orientation.w = 1.0;
        for (const auto& p : pts) {
            auto point = geometry_msgs::msg::Point { };
            point.x = p.x();
            point.y = p.y();
            point.z = p.z();
            m.points.emplace_back(point);
        }
    }
};

} // namespace

auto main() -> int {
    rclcpp::init(0, nullptr);

    auto node = std::make_shared<rclcpp::Node>("test_rune_ekf");
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/rmcs/auto_aim/rune_test/ekf", rclcpp::QoS { 10 });

    constexpr auto kDt = 0.01;

    auto engine      = std::mt19937 { 7 }; // NOLINT(cert-msc51-cpp)
    auto pixel_noise = std::normal_distribution<double> { 0.0, 1.0 };

    // 测试大符（转速正弦变化，点亮 2 块），验证正弦拟合
    auto rune      = MockRune { RuneModel::Kind::Big, engine };
    auto projector = Projector { engine, pixel_noise };

    auto config = RuneModel::Config { };
    auto model  = std::make_unique<RuneModel>(config);
    model->update_camera(
        std::array { Projector::kFx, 0.0, Projector::kCx, 0.0, Projector::kFy, Projector::kCy, 0.0,
            0.0, 1.0 },
        std::array { 0.0, 0.0, 0.0, 0.0, 0.0 });
    model->update_transform(Transform::kIdentity());

    auto elapsed      = 0.0;
    auto frame_index  = 0;
    auto initialized  = false;

    while (rclcpp::ok()) {
        elapsed += kDt;
        rune.step(kDt, elapsed);

        const auto stamp = node->now();

        // 构造观测：仅点亮扇叶可被检测
        auto bullseyes = std::vector<RuneBullseye> { };
        for (std::size_t i = 0; i < kBladeCount; ++i) {
            if (!rune.lit[i]) continue;
            if (auto b = projector.make_bullseye(rune, i, false)) {
                bullseyes.push_back(*b);
            }
        }
        auto icons = std::vector<RuneIcon> { };
        if (auto icon = projector.make_icon(rune)) icons.push_back(*icon);

        if (!initialized) {
            if (model->init(bullseyes, icons)) {
                initialized = true;
                std::println("[test] init ok, kind={}",
                    rune.kind == RuneModel::Kind::Big ? "Big" : "Small");
            }
        } else {
            model->predict(kDt);
            model->correct(bullseyes, icons);
        }

        if (initialized && frame_index % 20 == 0) {
            const auto st       = model->state();
            const auto err_c    = std::hypot(st.cx - MockRune::kCenter.x(),
                st.cy - MockRune::kCenter.y(), st.cz - MockRune::kCenter.z());
            const auto omega_tr = rune.omega(elapsed);

            auto blade_err = 0.0;
            const auto est = st.get_aimpoints();
            for (std::size_t i = 0; i < kBladeCount; ++i) {
                const auto pos_true = rune.blade_center(i);
                // 扇叶等价、index 仅内部标签，取到最近估计扇叶的距离
                auto nearest = std::numeric_limits<double>::max();
                for (const auto& p : est) {
                    const auto d = std::hypot(p.x - pos_true.x(), p.y - pos_true.y(),
                        p.z - pos_true.z());
                    nearest = std::min(nearest, d);
                }
                blade_err = std::max(blade_err, nearest);
            }

            std::println("t={:6.2f} center_err={:.3f} blade_err={:.3f} yaw_err={:+.1f}° omega_est={:+.3f} "
                         "omega_true={:+.3f} sin_a={:.3f}/{:.3f} sin_w={:.3f}/{:.3f}",
                elapsed, err_c, blade_err,
                util::normalize_angle(st.plane_yaw - MockRune::kPlaneYaw) * 180.0 / std::numbers::pi,
                st.rotation_speed, omega_tr,
                st.sin_a, rune.sin_a, st.sin_omega, rune.sin_omega);
        }

        // 3D 可视化
        {
            auto helper  = MarkerHelper { };
            helper.stamp = stamp;
            for (std::size_t i = 0; i < kBladeCount; ++i) {
                const auto pos = rune.blade_center(i);
                if (rune.lit[i]) {
                    helper.add_sphere("true_blade", pos, 0.10, 0.0, 1.0, 0.0, 1.0);
                    // 激活靶心才有十字，绘制其四个角点
                    const auto corners = rune.blade_corners(i);
                    helper.add_points("lit_corners",
                        std::span<const Eigen::Vector3d> { corners }, 0.04, 0.0, 1.0, 0.0, 1.0);
                } else {
                    helper.add_sphere("true_blade", pos, 0.08, 1.0, 1.0, 0.0, 0.25);
                }
            }
            if (initialized) {
                const auto aimpoints = model->state().get_aimpoints();
                for (const auto& p : aimpoints) {
                    const auto color_r = p.valid ? 1.0 : 0.0;
                    helper.add_sphere("est_blade", Eigen::Vector3d { p.x, p.y, p.z }, 0.06, color_r,
                        0.0, 1.0, 0.9);
                }
                helper.add_sphere("est_center",
                    Eigen::Vector3d { model->state().cx, model->state().cy, model->state().cz },
                    0.12, 0.0, 0.0, 1.0, 1.0);
            }
            marker_pub->publish(helper.array);
        }

        ++frame_index;
        rclcpp::sleep_for(std::chrono::milliseconds { 10 });
    }

    return 0;
}

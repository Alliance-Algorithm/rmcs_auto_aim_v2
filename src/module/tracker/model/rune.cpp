#include "rune.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/solve_pnp/pnp_solution.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <vector>

#include <eigen3/Eigen/Geometry>

using namespace rmcs;
using namespace rmcs::util;

namespace {

constexpr auto kBladeCount = std::size_t { 5 };
constexpr auto kBladeAngle = 2.0 * std::numbers::pi / 5.0; // 72°
constexpr auto kRadius     = kRuneGlobalRadius;

// 大符正弦目标函数参数范围（规则手册 5.5.2）
constexpr auto kBigOmegaMin = 1.884;
constexpr auto kBigOmegaMax = 2.000;
constexpr auto kBigAmpMin   = 0.780;
constexpr auto kBigAmpMax   = 1.045;
constexpr auto kBigBias     = 2.090; // b = kBigBias - a

} // namespace

auto RuneModel::State::transition(double seconds) -> void {
    if (kind == Kind::Big && sin_omega > 0.0) {
        // 大符：按拟合正弦目标函数外推（方向由 rotation_speed 符号确定）
        const auto magnitude = sin_a * std::sin(sin_omega * t_active) + (kBigBias - sin_a);
        const auto omega     = std::copysign(magnitude, rotation_speed);
        rotation_angle       = util::normalize_angle(rotation_angle + omega * seconds);
    } else {
        rotation_angle = util::normalize_angle(rotation_angle + rotation_speed * seconds);
    }
    t_active += seconds;
}

auto RuneModel::State::get_direction() const -> Point3d { return Point3d { cx, cy, cz }; }

auto RuneModel::State::get_rotation_speed() const -> double { return rotation_speed; }

auto RuneModel::State::get_aimpoints() const -> AimPoints {
    auto result = AimPoints { };
    result.reserve(kBladeCount);

    const auto cos_yaw = std::cos(plane_yaw);
    const auto sin_yaw = std::sin(plane_yaw);

    for (std::size_t i = 0; i < kBladeCount; ++i) {
        const auto phase = rotation_angle + static_cast<double>(i) * kBladeAngle;

        // 扇叶在符盘局部 Y-Z 平面内绕局部 X 轴旋转：local = (0, -r·sinφ, r·cosφ)
        const auto local_y = -kRadius * std::sin(phase);
        const auto local_z = +kRadius * std::cos(phase);

        // 绕世界 Z 轴按 plane_yaw 旋转后平移到中心
        const auto world_x = cx - sin_yaw * local_y;
        const auto world_y = cy + cos_yaw * local_y;
        const auto world_z = cz + local_z;

        result.emplace_back(world_x, world_y, world_z, lit[i]);
    }
    return result;
}

struct RuneModel::Impl {
    using StateVector = Eigen::Matrix<double, 6, 1>;
    using Covariance  = Eigen::Matrix<double, 6, 6>;
    using ObsVector   = Eigen::Matrix<double, 5, 1>;
    using ObsNoise    = Eigen::Matrix<double, 5, 5>;
    using ObsJacobian = Eigen::Matrix<double, 5, 6>;

    static constexpr auto kCx  = 0;
    static constexpr auto kCy  = 1;
    static constexpr auto kCz  = 2;
    static constexpr auto kYaw = 3;
    static constexpr auto kTh  = 4;
    static constexpr auto kW   = 5;

    Config config { };

    CameraFeature camera { };

    SingleRunePnpSolution pnp { };

    StateVector state     = StateVector::Zero();
    Covariance covariance = Covariance::Identity();

    RuneModel::Kind kind    = RuneModel::Kind::Small;
    double t_active         = 0.0;
    double sin_a            = 0.0;
    double sin_omega        = 0.0;
    std::array<bool, 5> lit = { };

    std::size_t update_count = 0;

    // 大符正弦拟合缓冲：记录 (t_active, ω) 序列
    struct OmegaSample {
        double t;
        double omega;
    };
    std::deque<OmegaSample> omega_history;

    // 单个符盘的一次观测（Odom 系）
    struct PageObservation {
        Eigen::Vector3d center;
        double plane_yaw;
        double phase; // 该扇叶绕符盘法向的相位
        bool active;
    };

    explicit Impl(const Config& cfg) noexcept
        : config { cfg } {
        reset_covariance();
    }

    // 从符盘朝向分解出 plane_yaw 与扇叶相位
    //
    // SingleRunePnpSolution 返回的 orientation 相对规范的 R_z(yaw)·R_x(phase)
    // 差一个固定的坐标轴置换 P（object frame 的轴角色与本模型约定不同）：
    //   PnP_R = R_z(yaw)·R_x(phase) · P
    // 故先右乘 Pᵀ 还原为规范旋转，再提取 plane_yaw（col0 法向）与相位。
    static auto decompose(const Eigen::Quaterniond& page) -> std::pair<double, double> {
        static const auto kAxisPermutation = [] {
            auto p = Eigen::Matrix3d { };
            // clang-format off
            p << 0, -1,  0,
                 0,  0, -1,
                 1,  0,  0;
            // clang-format on
            return p;
        }();

        const auto canonical =
            Eigen::Matrix3d { page.toRotationMatrix() * kAxisPermutation.transpose() };

        // 符盘法向 = 规范旋转的局部 X 轴
        const auto normal    = Eigen::Vector3d { canonical.col(0) };
        const auto plane_yaw = std::atan2(normal.y(), normal.x());

        // 去除 plane_yaw（绕世界 Z）后，剩余应为绕局部 X 的旋转
        const auto residual = Eigen::Matrix3d {
            Eigen::AngleAxisd { -plane_yaw, Eigen::Vector3d::UnitZ() }.toRotationMatrix()
            * canonical
        };
        const auto phase = std::atan2(residual(2, 1), residual(1, 1));

        return { plane_yaw, phase };
    }

    // 靶心最近的 R 标（2D）
    static auto nearest_icon(const RuneBullseye& bullseye, std::span<const RuneIcon> icons)
        -> std::optional<RuneIcon> {
        auto best     = std::optional<RuneIcon> { };
        auto best_dis = std::numeric_limits<double>::max();
        for (const auto& icon : icons) {
            const auto dx  = icon.center.x - bullseye.center.x;
            const auto dy  = icon.center.y - bullseye.center.y;
            const auto dis = dx * dx + dy * dy;
            if (dis < best_dis) {
                best_dis = dis;
                best     = icon;
            }
        }
        return best;
    }

    auto reset_covariance() noexcept -> void {
        auto diag = StateVector { };
        diag << 1.0, 1.0, 1.0, // cx cy cz
            0.4, // plane_yaw
            0.4, // θ
            10.0; // ω
        covariance = diag.asDiagonal();
    }

    auto process_noise() const noexcept -> Covariance {
        auto q        = Covariance::Zero().eval();
        q(kCx, kCx)   = config.process_noise_xy;
        q(kCy, kCy)   = config.process_noise_xy;
        q(kCz, kCz)   = config.process_noise_z;
        q(kYaw, kYaw) = config.process_noise_plane_yaw;
        q(kTh, kTh)   = config.process_noise_angle;
        q(kW, kW)     = config.process_noise_speed;
        return q;
    }

    auto observation_noise() const noexcept -> ObsNoise {
        auto r  = ObsNoise::Zero().eval();
        r(0, 0) = config.observation_noise_xy;
        r(1, 1) = config.observation_noise_xy;
        r(2, 2) = config.observation_noise_z;
        r(3, 3) = config.observation_noise_plane_yaw;
        r(4, 4) = config.observation_noise_angle;
        return r;
    }

    // 相机(ros 轴)系 → Odom 系
    auto to_odom(const Eigen::Vector3d& p_cam) const -> Eigen::Vector3d {
        const auto q = camera.orientation.make<Eigen::Quaterniond>();
        const auto t = camera.translation.make<Eigen::Vector3d>();
        return q * p_cam + t;
    }
    auto to_odom(const Eigen::Quaterniond& r_cam) const -> Eigen::Quaterniond {
        const auto q = camera.orientation.make<Eigen::Quaterniond>();
        return q * r_cam;
    }

    auto make_observations(std::span<const RuneBullseye> bullseyes, std::span<const RuneIcon> icons)
        -> std::vector<PageObservation> {
        auto result = std::vector<PageObservation> { };

        for (const auto& bullseye : bullseyes) {
            const auto icon = nearest_icon(bullseye, icons);
            if (!icon) continue;

            pnp.input.center  = bullseye.center;
            pnp.input.icon    = icon->center;
            pnp.input.corners = bullseye.corners;

            if (!pnp.solve()) continue;

            const auto center_cam = pnp.result.translation.make<Eigen::Vector3d>();
            const auto page_cam   = pnp.result.orientation.make<Eigen::Quaterniond>();

            const auto center_odom      = to_odom(center_cam);
            const auto page_odom        = to_odom(page_cam);
            const auto [plane_yaw, phi] = decompose(page_odom);

            result.push_back(PageObservation {
                .center    = center_odom,
                .plane_yaw = plane_yaw,
                .phase     = phi,
                .active    = bullseye.active,
            });
        }
        return result;
    }

    // 将观测相位吸附到最近的扇叶 index
    auto adhere_index(double phase) const -> std::size_t {
        auto best     = std::size_t { 0 };
        auto best_err = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < kBladeCount; ++i) {
            const auto anchor = state[kTh] + static_cast<double>(i) * kBladeAngle;
            const auto err    = std::abs(util::normalize_angle(phase - anchor));
            if (err < best_err) {
                best_err = err;
                best     = i;
            }
        }
        return best;
    }

    auto init(std::span<const RuneBullseye> bullseyes, std::span<const RuneIcon> icons) -> bool {
        const auto observations = make_observations(bullseyes, icons);
        if (observations.empty()) return false;

        // 中心与朝向取均值
        auto center    = Eigen::Vector3d::Zero().eval();
        auto yaw_sum_s = 0.0;
        auto yaw_sum_c = 0.0;
        for (const auto& obs : observations) {
            center += obs.center;
            yaw_sum_s += std::sin(obs.plane_yaw);
            yaw_sum_c += std::cos(obs.plane_yaw);
        }
        center /= static_cast<double>(observations.size());
        const auto plane_yaw = std::atan2(yaw_sum_s, yaw_sum_c);

        // 以首个观测为 index 0 参考相位
        const auto theta = observations.front().phase;

        state.setZero();
        state[kCx]  = center.x();
        state[kCy]  = center.y();
        state[kCz]  = center.z();
        state[kYaw] = plane_yaw;
        state[kTh]  = theta;
        state[kW]   = 0.0;

        reset_covariance();

        // 点亮状态 + 未激活数量分类
        lit.fill(false);
        auto inactive_count = std::size_t { 0 };
        for (const auto& obs : observations) {
            if (!obs.active) {
                lit[adhere_index(obs.phase)] = true;
                ++inactive_count;
            }
        }
        kind = inactive_count >= 2 ? RuneModel::Kind::Big : RuneModel::Kind::Small;

        t_active  = 0.0;
        sin_a     = 0.0;
        sin_omega = 0.0;
        omega_history.clear();
        update_count = 0;

        return true;
    }

    auto predict(double dt) -> void {
        state[kTh] = util::normalize_angle(state[kTh] + state[kW] * dt);

        auto jacobian     = Covariance { Covariance::Identity() };
        jacobian(kTh, kW) = dt;

        covariance = jacobian * covariance * jacobian.transpose() + process_noise();

        t_active += dt;
    }

    auto update(const PageObservation& obs, std::size_t index) -> void {
        const auto theta_obs =
            util::normalize_angle(obs.phase - static_cast<double>(index) * kBladeAngle);

        auto measurement = ObsVector { };
        measurement << obs.center.x(), obs.center.y(), obs.center.z(), obs.plane_yaw, theta_obs;

        auto predicted = ObsVector { };
        predicted << state[kCx], state[kCy], state[kCz], state[kYaw], state[kTh];

        auto innovation = ObsVector { measurement - predicted };
        innovation(3)   = util::normalize_angle(innovation(3));
        innovation(4)   = util::normalize_angle(innovation(4));

        auto jacobian     = ObsJacobian::Zero().eval();
        jacobian(0, kCx)  = 1.0;
        jacobian(1, kCy)  = 1.0;
        jacobian(2, kCz)  = 1.0;
        jacobian(3, kYaw) = 1.0;
        jacobian(4, kTh)  = 1.0;

        const auto obs_noise = observation_noise();

        const auto innovation_covariance = jacobian * covariance * jacobian.transpose() + obs_noise;
        const auto kalman_gain =
            covariance * jacobian.transpose() * innovation_covariance.inverse();

        auto posterior = StateVector { state + kalman_gain * innovation };
        if (posterior.hasNaN() || !posterior.allFinite()) return;

        posterior[kYaw] = util::normalize_angle(posterior[kYaw]);
        posterior[kTh]  = util::normalize_angle(posterior[kTh]);

        const auto complement = Covariance::Identity() - kalman_gain * jacobian;
        auto posterior_cov    = Covariance { complement * covariance * complement.transpose()
            + kalman_gain * obs_noise * kalman_gain.transpose() };
        posterior_cov         = 0.5 * (posterior_cov + posterior_cov.transpose());

        state      = posterior;
        covariance = posterior_cov;
    }

    auto correct(std::span<const RuneBullseye> bullseyes, std::span<const RuneIcon> icons) -> void {
        const auto observations = make_observations(bullseyes, icons);
        if (observations.empty()) return;

        lit.fill(false);
        auto inactive_count = std::size_t { 0 };
        for (const auto& obs : observations) {
            const auto index = adhere_index(obs.phase);
            update(obs, index);
            if (!obs.active) {
                lit[index] = true;
                ++inactive_count;
            }
        }

        if (kind == RuneModel::Kind::Small && inactive_count >= 2) {
            kind = RuneModel::Kind::Big;
        }

        record_omega();
        fit_sinusoid();

        update_count += 1;
    }

    auto record_omega() -> void {
        static constexpr auto kOmegaHistoryCapacity = std::size_t { 200 };

        omega_history.push_back({ t_active, state[kW] });
        if (omega_history.size() > kOmegaHistoryCapacity) {
            omega_history.pop_front();
        }
    }

    // 大符正弦拟合：ω(t) = a·sin(ω_sin·t) + (kBigBias - a)
    // 固定 ω_sin 时对 a 做线性最小二乘，网格搜索 ω_sin 取残差最小
    auto fit_sinusoid() -> void {
        constexpr auto kGridSteps  = 24;
        constexpr auto kMinSamples = std::size_t { 20 };
        constexpr auto kMinDenom   = 1e-9;

        if (kind != RuneModel::Kind::Big) return;
        if (omega_history.size() < kMinSamples) return;

        auto best_omega    = 0.0;
        auto best_a        = 0.0;
        auto best_residual = std::numeric_limits<double>::max();

        for (auto step = 0; step <= kGridSteps; ++step) {
            const auto omega_sin = kBigOmegaMin
                + (kBigOmegaMax - kBigOmegaMin) * static_cast<double>(step) / kGridSteps;

            // 固定 ω_sin，用 s = sin(ω_sin·t) - 1 对 y = |ω| - b0 做线性回归 y = a·s
            auto a = 0.0;
            {
                auto sum_ss = 0.0;
                auto sum_sy = 0.0;
                for (const auto& sample : omega_history) {
                    const auto s = std::sin(omega_sin * sample.t) - 1.0;
                    const auto y = std::abs(sample.omega) - kBigBias;
                    sum_ss += s * s;
                    sum_sy += s * y;
                }
                if (sum_ss < kMinDenom) continue;
                a = std::clamp(sum_sy / sum_ss, kBigAmpMin, kBigAmpMax);
            }

            // 用拟合参数评估残差
            auto residual = 0.0;
            for (const auto& sample : omega_history) {
                const auto predicted = a * std::sin(omega_sin * sample.t) + (kBigBias - a);
                const auto diff      = std::abs(sample.omega) - predicted;
                residual += diff * diff;
            }

            if (residual < best_residual) {
                best_residual = residual;
                best_omega    = omega_sin;
                best_a        = a;
            }
        }

        sin_a     = best_a;
        sin_omega = best_omega;
    }

    auto converge() const -> bool {
        constexpr auto kMinUpdate = std::size_t { 10 };
        constexpr auto kMaxCovXY  = 0.05;
        constexpr auto kMaxCovYaw = 0.05;

        if (update_count < kMinUpdate) return false;
        if (covariance(kCx, kCx) > kMaxCovXY) return false;
        if (covariance(kCy, kCy) > kMaxCovXY) return false;
        if (covariance(kTh, kTh) > kMaxCovYaw) return false;
        return true;
    }

    auto diverged() const -> bool {
        return std::ranges::any_of(
            std::array {
                covariance(kCx, kCx) > 100.0,
                covariance(kCy, kCy) > 100.0,
                std::abs(state[kCx]) > 20.0,
                std::abs(state[kCy]) > 20.0,
                std::abs(state[kCz]) > 5.0,
                std::abs(state[kW]) > 10.0 * std::numbers::pi,
                state.hasNaN(),
                !state.allFinite(),
            },
            std::identity { });
    }

    auto make_state() const -> State {
        auto s = State { };

        s.cx             = state[kCx];
        s.cy             = state[kCy];
        s.cz             = state[kCz];
        s.plane_yaw      = state[kYaw];
        s.rotation_angle = state[kTh];
        s.rotation_speed = state[kW];

        s.kind      = kind;
        s.t_active  = t_active;
        s.sin_a     = sin_a;
        s.sin_omega = sin_omega;
        s.lit       = lit;

        return s;
    }
};

RuneModel::RuneModel(const Config& cfg) noexcept
    : pimpl { std::make_unique<Impl>(cfg) } { }

RuneModel::~RuneModel() noexcept = default;

auto RuneModel::update_camera(std::array<double, 9> matrix, std::array<double, 5> coeff) noexcept
    -> void {
    pimpl->camera.from(matrix);
    pimpl->camera.from(coeff);
    pimpl->pnp.input.cam = pimpl->camera;
}

auto RuneModel::update_transform(const Transform& t) noexcept -> void {
    pimpl->camera.translation = t.translation;
    pimpl->camera.orientation = t.orientation;
    pimpl->pnp.input.cam      = pimpl->camera;
}

auto RuneModel::init(
    std::span<const RuneBullseye> bullseyes, std::span<const RuneIcon> icons) noexcept -> bool {
    return pimpl->init(bullseyes, icons);
}

auto RuneModel::predict(double dt) noexcept -> void { pimpl->predict(dt); }

auto RuneModel::correct(
    std::span<const RuneBullseye> bullseyes, std::span<const RuneIcon> icons) noexcept -> void {
    pimpl->correct(bullseyes, icons);
}

auto RuneModel::converge() const -> bool { return pimpl->converge(); }

auto RuneModel::diverged() const -> bool { return pimpl->diverged(); }

auto RuneModel::state() const noexcept -> State { return pimpl->make_state(); }

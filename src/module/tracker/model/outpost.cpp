#include "outpost.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/outpost.hpp"
#include "utility/robot/constant.hpp"

#include <eigen3/Eigen/Geometry>

using namespace rmcs;
using namespace rmcs::util;

constexpr auto kRadius     = kOutpostRadius;
constexpr auto kPitch      = kPredictedOutpostArmorPitch;
constexpr auto kHeightStep = kOutpostArmorHeightStep;
constexpr auto kAngleStep  = 2.0 * std::numbers::pi / 3.0;

constexpr auto kOffsetTable = std::array {
    std::tuple { 0.0 * kAngleStep, -0.0 * kHeightStep },
    std::tuple { 1.0 * kAngleStep, -1.0 * kHeightStep },
    std::tuple { 2.0 * kAngleStep, -2.0 * kHeightStep },
};

auto OutpostModel::State::transition(double seconds) -> void {
    rotation_angle = util::normalize_angle(rotation_angle + rotation_speed * seconds);
}

auto OutpostModel::State::get_direction() const -> Point3d { return Point3d { x, y, z }; }

auto OutpostModel::State::get_rotation_speed() const -> double { return rotation_speed; }

auto OutpostModel::State::get_aimpoints() const -> std::vector<Point3d> {
    const auto& [ref_yaw_off, ref_h_off] = kOffsetTable[index];

    auto result = std::vector<Point3d> { };
    for (int i = 0; i < 3; ++i) {
        const auto& [yaw_off, h_off] = kOffsetTable[i];

        const auto pa = util::normalize_angle(rotation_angle + (yaw_off - ref_yaw_off));
        const auto px = x + kRadius * std::cos(pa);
        const auto py = y + kRadius * std::sin(pa);
        const auto pz = z + (h_off - ref_h_off);

        result.emplace_back(px, py, pz);
    }
    return result;
}

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

        Context() noexcept = default;

        auto set_noise(const Config& config) noexcept {
            noise_process.diagonal() << config.process_noise_xy, config.process_noise_xy,
                config.process_noise_z, config.process_noise_speed, config.process_noise_angle;
            noise_observation.diagonal() << config.observation_noise_xy,
                config.observation_noise_xy, config.observation_noise_z,
                config.observation_noise_yaw;
        }

        auto reset_covariance() noexcept {
            auto diag = Eigen::Matrix<double, 5, 1> { };
            diag << 1.0, 1.0, 1.0, 64.0, 0.4;
            posteriors_covariance = diag.asDiagonal();
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

    struct HeightBuffer {
        double factor = 0.1;

        std::array<double, 3> data;
        int last_sign = 0;

        std::size_t index = 0;

        HeightBuffer() noexcept { std::ranges::fill(data, kNaN); }

        auto go_next(int sign) noexcept {
            last_sign = sign;
            index     = (index + sign + data.size()) % data.size();
        }

        auto update(double height) noexcept {
            data[index] = std::isnan(data[index]) //
                ? height
                : data[index] * (1. - factor) + height * factor;
        }
        auto clean_oldest() noexcept {
            data[(index - 2 * last_sign + data.size()) % data.size()] = kNaN;
        }

        auto all_observed() const noexcept -> bool {
            return !std::isnan(data[0]) && !std::isnan(data[1]) && !std::isnan(data[2]);
        }

        auto level_at(std::size_t pos) const noexcept {
            const auto height = data[pos];

            const auto l_height = data[(pos - 1 + data.size()) % data.size()];
            const auto r_height = data[(pos + 1 + data.size()) % data.size()];

            /// @NOTE: is_lower / is_upper 在遇到 NaN 时返回 true 的设计
            ///
            ///  data 中尚未观测的装甲板对应的高度值为 NaN。当参与比较的任一
            ///  值为 NaN 时，比较结果被认定为真——该条件自动满足，不参与判定。
            ///  因此当前板的 level 仅由已知邻居的真实比较确定。
            ///
            ///  - 三块装甲板都被观测到后：无 NaN，所有比较均为真实比较，
            ///    level() 自然给出当前板正确的序号。
            ///
            ///  - 只观测到两块装甲板时：当前板与已知邻居的真实比较仍成立，
            ///    level() 能保证已观测两块板间的相对高度正确，足以支持 EKF
            ///    的观测更新。但整组序号可能存在歧义：例如仅观测到两块板且
            ///    高度差为 1 step 时，无法区分是 UP→MID 还是 MID→LOW（两者
            ///    相对高度差一致），当前板的绝对序号可能错误，但不影响 EKF
            ///    迭代；待第三块板被观测到后，整组序号即被完全确定。
            constexpr auto is_lower = [](double z, double other) {
                return (std::isnan(z) || std::isnan(other)) ? true : z < other;
            };
            constexpr auto is_upper = [](double z, double other) {
                return (std::isnan(z) || std::isnan(other)) ? true : z > other;
            };

            // 带 '*' 的即为观测到的装甲板
            /*--*/ if (is_upper(height, l_height) && is_upper(height, r_height)) {
                ///     [*]
                ///         [ ]
                /// [ ]
                return 0;
            } else if (is_lower(height, l_height) && is_upper(height, r_height)) {
                /// [ ]
                ///     [*]
                ///         [ ]
                return 1; // 中
            } else if (is_lower(height, l_height) && is_lower(height, r_height)) {
                ///         [ ]
                /// [ ]
                ///     [*]
                return 2; // 低
            }
            return 0;
        }

        auto level() const noexcept { return level_at(index); }
    };
    HeightBuffer height_buff { };

    int abs_ref_index = 0; // 参考板绝对序号 0/1/2（第一块观测板，三块板齐后纠正）
    int rel_see_index = 0; // 当前板相对 ref_index 的位置 -1/0/+1

    ArmorGenre armor_genre { DeviceId::OUTPOST };
    ArmorColor armor_color { ArmorColor::DARK };

    // 切板检测：存储上一帧观测向量（xyz + yaw）
    // 构造时初始化，合法帧每帧更新；噪声帧跳过不更新
    ObservationVector last_observation { };

    static constexpr int kSignConfirmCount = 2;

    // 旋转方向确认：连续 2 次同方向切板后锁定
    int rotation_sign       = 0; // +1/-1，0=未确定
    int pending_sign        = 0; // 候选方向
    int sign_evidence_count = 0; // 候选方向连续命中计数

    std::size_t update_count = 0;

    Context context;
    Config config { };

    auto configure(const Config& cfg) noexcept {
        config = cfg;
        context.set_noise(config);
        context.reset_covariance();
    }

private:
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

public:
    explicit Impl(const Armor3d& armor) noexcept {
        constexpr auto pitch = kPredictedOutpostArmorPitch;

        // 装甲板元信息
        armor_genre = armor.genre;
        armor_color = armor.color;

        const auto translation = armor.translation.make<Eigen::Vector3d>();
        const auto orientation = armor.orientation.make<Eigen::Quaterniond>();

        const auto backward     = Eigen::Vector3d { orientation * Eigen::Vector3d::UnitX() };
        const auto armor2center = Eigen::Vector3d {
            Eigen::AngleAxisd { -pitch, orientation * Eigen::Vector3d::UnitY() } * backward
        };
        const auto center = Eigen::Vector3d { translation + kRadius * armor2center };

        context.update_center(center);
        context.posteriors_state[3] = 0.0;
        context.posteriors_state[4] =
            util::normalize_angle(std::atan2(-armor2center.y(), -armor2center.x()));

        context.set_noise(config);
        context.reset_covariance();

        // 初始化 last_observation：从 EKF 后验 state 取（与 correct() 末尾更新方式一致）
        // state[4] = atan2(-atc.y, -atc.x) = center→armor 方向角度
        last_observation << context.posteriors_state[0], context.posteriors_state[1],
            context.posteriors_state[2], context.posteriors_state[4];

        // 初始化 HeightBuff：第一块板存入 data[0]（index 初始为 0）
        height_buff.update(center.z());
    }

    auto predict(double dt) noexcept -> void {
        const auto prior_state      = predict_state(dt, context.posteriors_state);
        const auto prior_covariance = predict_covariance(dt, context.posteriors_covariance);

        context.posteriors_state      = prior_state;
        context.posteriors_covariance = prior_covariance;
    }

    auto correct(const Armor3d& armor) noexcept -> void {
        constexpr auto kPitch = kPredictedOutpostArmorPitch;

        // 装甲板元信息
        armor_genre = armor.genre;
        armor_color = armor.color;

        // 观测向量构造
        auto observation = ObservationVector { };
        {
            const auto orientation = armor.orientation.make<Eigen::Quaterniond>();
            const auto translation = armor.translation.make<Eigen::Vector3d>();

            const auto backward     = Eigen::Vector3d { orientation * Eigen::Vector3d::UnitX() };
            const auto armor2center = Eigen::Vector3d {
                Eigen::AngleAxisd { -kPitch, orientation * Eigen::Vector3d::UnitY() } * backward
            };

            const auto center = Eigen::Vector3d { translation + kRadius * armor2center };

            const auto armor_yaw =
                util::normalize_angle(std::atan2(-armor2center.y(), -armor2center.x()));

            observation << center.x(), center.y(), center.z(), armor_yaw;
        }

        // 切板检测：判定是否发生真实切板
        // 判定依据：yaw 跳变大于配置阈值，且 center.xy 偏移落在 10cm 内
        {
            constexpr auto kPlateSwitchXYTol = 0.10; // 10cm

            const auto delta_yaw_obs = util::normalize_angle(observation[3] - last_observation[3]);
            const auto abs_delta     = std::abs(delta_yaw_obs);

            if (abs_delta > util::deg2rad(config.plate_switch_yaw_min)) {
                // 候选切板：用 center.xy 偏移做位置验证
                const double xy_error = std::hypot(
                    observation[0] - last_observation[0], observation[1] - last_observation[1]);

                if (xy_error > kPlateSwitchXYTol) {
                    // 位置验证不通过：噪声，丢弃
                    return;
                }

                // 真实切板：从观测符号判断方向，直接取装配标准值
                const auto in_right = delta_yaw_obs > 0.0;

                // 旋转方向确认：连续 2 次同方向切板后锁定
                const auto sign = in_right ? +1 : -1;
                if (rotation_sign == 0) {
                    if (sign == pending_sign) {
                        sign_evidence_count++;
                    } else {
                        pending_sign        = sign;
                        sign_evidence_count = 1;
                    }
                    if (sign_evidence_count >= kSignConfirmCount) {
                        rotation_sign = pending_sign;
                    }
                }
                const auto go_sign = rotation_sign != 0 ? rotation_sign : sign;

                // 更新 HeightBuff
                height_buff.go_next(go_sign);
                height_buff.update(observation[2]);

                // obs_index：相对 ref_index 的位置 -1/0/+1
                rel_see_index += sign;
                if (rel_see_index > +1) rel_see_index -= 3;
                if (rel_see_index < -1) rel_see_index += 3;

                // 三块板齐后纠正 abs_ref_index（绝对序号确定）
                if (height_buff.all_observed()) {
                    abs_ref_index = (height_buff.level() - rel_see_index + 3) % 3;

                    height_buff.clean_oldest();
                }
            }
        }

        // 补偿观测：将当前板观测变换回参考板坐标系
        // obs_index = 0 无需补偿；否则用 outpost_relative_height 从高度符号判断
        auto delta_yaw    = 0.0;
        auto delta_height = 0.0;
        if (rel_see_index != 0) {
            const auto in_right = rel_see_index > 0;
            const auto in_upper = observation[2] > context.posteriors_state[2];
            delta_yaw    = in_right ? +std::numbers::pi * 2.0 / 3.0 : -std::numbers::pi * 2.0 / 3.0;
            delta_height = outpost_relative_height(in_right, in_upper);
        }
        observation[2] -= delta_height;
        observation[3] = util::normalize_angle(observation[3] - delta_yaw);

        const auto prior_state      = context.posteriors_state;
        const auto prior_covariance = context.posteriors_covariance;

        const auto innovation  = measure_innovation(observation, prior_state);
        const auto kalman_gain = calculate_kalman_gain(prior_covariance);
        const auto jacobian    = make_observation_jacobian();

        const auto [posterior_state, posterior_covariance] =
            a_posteriori_update(prior_state, prior_covariance, kalman_gain, innovation, jacobian);

        context.posteriors_state      = posterior_state;
        context.posteriors_covariance = posterior_covariance;

        // 用 EKF 后验 state 作为 last_observation（低噪声平滑值）
        // state 跟踪参考板，需反补偿回当前板坐标系，使下一帧切板检测正确
        last_observation << context.posteriors_state[0], context.posteriors_state[1],
            context.posteriors_state[2] + delta_height,
            util::normalize_angle(context.posteriors_state[4] + delta_yaw);

        update_count += 1;
    }

    auto at(std::uint8_t index) const noexcept -> Armor3d {
        // EKF state 跟踪参考板（ref_index），求 index 相对 ref_index 的偏移
        const auto delta_yaw = util::normalize_angle(
            std::get<0>(kOffsetTable[index]) - std::get<0>(kOffsetTable[abs_ref_index]));
        const auto delta_height =
            std::get<1>(kOffsetTable[index]) - std::get<1>(kOffsetTable[abs_ref_index]);

        // center→armor 方向的 yaw（state[4] 的约定）
        const auto center2armor_yaw =
            util::normalize_angle(context.posteriors_state[4] + delta_yaw);

        // 装甲板位置 = 旋转中心 + radius * center→armor 方向
        const auto armor_pos = Eigen::Vector3d {
            context.posteriors_state[0] + kRadius * std::cos(center2armor_yaw),
            context.posteriors_state[1] + kRadius * std::sin(center2armor_yaw),
            context.posteriors_state[2] + delta_height,
        };

        // 装甲板朝向约定：orientation 的 X 轴 = armor→center
        const auto orientation = Eigen::Quaterniond {
            Eigen::AngleAxisd { center2armor_yaw + std::numbers::pi, Eigen::Vector3d::UnitZ() }
                * Eigen::AngleAxisd { kPitch, Eigen::Vector3d::UnitY() },
        };

        return Armor3d {
            .genre = armor_genre,
            .color = armor_color,
            .id    = index,

            .translation = Translation { armor_pos },
            .orientation = Orientation { orientation },
        };
    }

    auto converge() const -> bool {
        constexpr auto kMinUpdate = std::size_t { 5 };
        constexpr auto kMaxCovXY  = 0.01;
        constexpr auto kMaxCovYaw = 0.01;

        if (update_count < kMinUpdate) return false;

        const auto& cov = context.posteriors_covariance;
        if (cov(0, 0) > kMaxCovXY) return false;
        if (cov(1, 1) > kMaxCovXY) return false;
        if (cov(4, 4) > kMaxCovYaw) return false;

        return true;
    }
};

OutpostModel::OutpostModel(const Armor3d& armor) noexcept
    : pimpl { std::make_unique<Impl>(armor) } { }

OutpostModel::~OutpostModel() noexcept = default;

auto OutpostModel::state() noexcept -> State {
    auto s  = pimpl->context.get_state();
    s.index = static_cast<std::uint8_t>(pimpl->abs_ref_index);
    return s;
}

auto OutpostModel::configure(const Config& cfg) noexcept -> void { pimpl->configure(cfg); }

auto OutpostModel::predict(double dt) noexcept -> void { pimpl->predict(dt); }

auto OutpostModel::correct(const Armor3d& armor) noexcept -> void { pimpl->correct(armor); }

auto OutpostModel::full() const -> std::array<Armor3d, 3> {
    return std::array {
        pimpl->at(0),
        pimpl->at(1),
        pimpl->at(2),
    };
}
auto OutpostModel::current() const -> Armor3d {
    return pimpl->at((pimpl->abs_ref_index + pimpl->rel_see_index + 3) % 3);
}

auto OutpostModel::converge() const -> bool { return pimpl->converge(); }

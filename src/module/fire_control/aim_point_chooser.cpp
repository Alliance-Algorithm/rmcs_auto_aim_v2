#include "aim_point_chooser.hpp"

#include "utility/math/conversion.hpp"
#include <optional>
#include <span>

using namespace rmcs::fire_control;

struct AimPointChooser::Impl {

    double coming_angle { 60 / 57.3 };  // rad
    double leaving_angle { 20 / 57.3 }; // rad

    double outpost_coming_angle { 70 / 57.3 };  // rad
    double outpost_leaving_angle { 30 / 57.3 }; // rad

    double angular_velocity_threshold { 2 }; // rad/s

    int last_chosen_id { -1 };

    auto initialize(Config const& config) noexcept -> void {
        coming_angle  = config.coming_angle;
        leaving_angle = config.leaving_angle;

        outpost_coming_angle  = config.outpost_coming_angle;
        outpost_leaving_angle = config.outpost_leaving_angle;

        angular_velocity_threshold = config.angular_velocity_threshold;
    }

    auto choose_armor(std::span<Armor3D const> armors, Eigen::Vector<double, 11> const& ekf_x)
        -> std::optional<Armor3D> {
        if (armors.empty()) {
            last_chosen_id = -1;
            return std::nullopt;
        }

        const auto car_y = ekf_x[2], car_x = ekf_x[0];
        const auto center_yaw       = std::atan2(car_y, car_x);
        const auto angular_velocity = ekf_x[7];

        struct ArmorCandidate {
            int index;
            double delta_yaw;
            double score; // 分数越低越好
        };

        auto candidates = std::array<ArmorCandidate, 4> {};
        const auto n    = std::min(armors.size(), candidates.size());

        for (size_t id = 0; id < n; ++id) {
            auto orientation = Eigen::Quaterniond {};
            armors[id].orientation.copy_to(orientation);

            const auto ypr          = util::eulers(orientation);
            const auto yaw_in_world = ypr[0];

            candidates.at(id) = { static_cast<int>(id),
                util::normalize_angle(yaw_in_world - center_yaw), 0. };
        }

        auto chosen_id = int { -1 };

        // --- 模式 A: 非小陀螺模式 (低速旋转) ---
        // 判断标准：角速度绝对值小于阈值 且 不是前哨站
        if ((std::abs(angular_velocity) < angular_velocity_threshold)
            && (armors.front().genre != DeviceId::OUTPOST)) {
            for (auto& candidate : candidates) {
                candidate.score = std::abs(candidate.delta_yaw);
                // 【锁定逻辑】如果该板是上一帧选中的，给它一个“粘滞补偿” (减分即优选)
                if (candidate.index == last_chosen_id)
                    candidate.score -= util::deg2rad(8); // 约 8 度的优先权，防止微小跳变导致换板
            }

            auto valid_it = std::ranges::min_element(
                candidates | std::views::take(n), {}, [](auto const& candidate) {
                    return (std::abs(candidate.delta_yaw) > util::deg2rad(90)) ? 1e5
                                                                               : candidate.score;
                });
            if (std::abs(valid_it->delta_yaw) < util::deg2rad(90)) chosen_id = valid_it->index;
        }
        // --- 模式 B: 小陀螺模式 (快速旋转) ---
        else {
            auto genre         = armors.front().genre;
            auto _coming_angle = (genre == DeviceId::OUTPOST) ? outpost_coming_angle : coming_angle;
            auto _leaving_angle =
                (genre == DeviceId::OUTPOST) ? outpost_leaving_angle : leaving_angle;

            for (auto& candidate : candidates) {
                candidate.score = std::abs(candidate.delta_yaw);
                // 判断旋转方向，给予“顺势”补偿
                // 如果 omega > 0 (逆时针)，板从右侧 (delta > 0) 进入。
                // 我们给正处于“迎面而来”位置的板减分（优选）
                bool is_incoming = (angular_velocity > 0 && candidate.delta_yaw > 0)
                    || (angular_velocity < 0 && candidate.delta_yaw < 0);

                if (is_incoming) candidate.score -= 0.2;
                // 【锁定逻辑】强粘滞：小陀螺下换板的代价更高
                if (candidate.index == last_chosen_id) {
                    candidate.score -= util::deg2rad(17); // 约 17 度的优先权
                }
                // 剔除已经快要转没的板 (Leaving Angle)
                if ((angular_velocity > 0 && candidate.delta_yaw < -_leaving_angle)
                    || (angular_velocity < 0 && candidate.delta_yaw > _leaving_angle)) {
                    candidate.score += 10.0; // 极大的惩罚项，强制换到下一块进入的板
                }

                auto it = std::ranges::min_element(
                    candidates | std::views::take(n), std::ranges::less {}, &ArmorCandidate::score);

                if (std::abs(it->delta_yaw) < _coming_angle) chosen_id = it->index;
            }
        }

        if (chosen_id != -1) {
            last_chosen_id = chosen_id;
            return { armors[chosen_id] };
        }

        last_chosen_id = -1;
        return std::nullopt;
    }
};

AimPointChooser::AimPointChooser() noexcept
    : pimpl { std::make_unique<Impl>() } { }

AimPointChooser::~AimPointChooser() noexcept = default;

auto AimPointChooser::initialize(Config const& config) noexcept -> void {
    return pimpl->initialize(config);
}

auto AimPointChooser::choose_armor(std::span<Armor3D const> armors,
    Eigen::Vector<double, 11> const& ekf_x) -> std::optional<Armor3D> {
    return pimpl->choose_armor(armors, ekf_x);
}

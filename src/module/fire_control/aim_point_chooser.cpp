#include "aim_point_chooser.hpp"

#include <cmath>
#include <tuple>
#include <vector>

#include "utility/math/conversion.hpp"

#include <cmath>
#include <tuple>
#include <vector>

using namespace rmcs::fire_control;

struct AimPointChooser::Impl {
    auto initialize(Config const& config) noexcept -> void { config_ = config; }

    auto choose_armor(std::span<Armor3D const> armors, Eigen::Vector3d const& center_position,
        double angular_velocity) -> std::optional<Armor3D> {
        if (armors.empty()) {
            last_chosen_armor_id.reset();
            last_chosen_armor_id.reset();
            return std::nullopt;
        }

        const auto center_yaw    = std::atan2(center_position.y(), center_position.x());
        const auto is_outpost    = armors.front().genre == DeviceId::OUTPOST;
        auto const active_coming = is_outpost ? config_.outpost_coming_angle : config_.coming_angle;
        auto const active_leaving =
            is_outpost ? config_.outpost_leaving_angle : config_.leaving_angle;

        auto candidate_evals = std::vector<CandidateEval>(armors.size());

        const auto yaw = [&](size_t index) {
            auto orientation = Eigen::Quaterniond {};
            armors[index].orientation.copy_to(orientation);
            return util::eulers(orientation)[0];
        };

        const auto in_window = [&](double delta_yaw) {
            auto const abs_delta  = std::abs(delta_yaw);
            auto const in_coming  = abs_delta <= active_coming;
            auto const in_leaving = (angular_velocity > 0.0) ? (delta_yaw <= active_leaving)
                : (angular_velocity < 0.0)                   ? (delta_yaw >= -active_leaving)
                                                             : true;
            return in_coming && in_leaving;
        };

        { // 1) 候选评估
            for (size_t index = 0; index < armors.size(); ++index) {
                auto const delta_yaw   = util::normalize_angle(yaw(index) - center_yaw);
                candidate_evals[index] = {
                    .delta_yaw = delta_yaw,
                    .in_window = in_window(delta_yaw),
                };
            }
        }

        const auto priority_key = [&](size_t index) {
            // 优先级：
            // 1) abs_delta：角误差更小优先
            // 2) last_penalty：上一帧目标优先（is_last -> 0，其它 -> 1）
            // 3) id：稳定排序
            // 4) index：最终兜底，保证结果确定性
            auto const abs_delta = std::abs(candidate_evals[index].delta_yaw);
            auto const id        = armors[index].id;
            auto const is_last = last_chosen_armor_id.has_value() && (id == *last_chosen_armor_id);
            auto const last_penalty = is_last ? 0 : 1;
            return std::tuple { abs_delta, last_penalty, id, index };
        };

        auto best_idx = std::optional<size_t> {};
        auto last_idx = std::optional<size_t> {};

        {
            // 2) 最优筛选（仅角度窗口内）并定位上次目标
            for (size_t index = 0; index < armors.size(); ++index) {
                if (last_chosen_armor_id.has_value()
                    && (armors[index].id == *last_chosen_armor_id)) {
                    last_idx = index;
                }

                if (!candidate_evals[index].in_window) continue;

                if (!best_idx.has_value() || (priority_key(index) < priority_key(*best_idx))) {
                    best_idx = index;
                }
            }
        }

        if (!best_idx.has_value()) {
            last_chosen_armor_id.reset();
            return std::nullopt;
        }

        {
            // 3) 切换抖动抑制
            if (last_idx.has_value() && (*last_idx != *best_idx)) {
                auto const last_abs    = std::abs(candidate_evals[*last_idx].delta_yaw);
                auto const best_abs    = std::abs(candidate_evals[*best_idx].delta_yaw);
                auto const improvement = last_abs - best_abs;
                if (improvement < min_switch_improvement_angle) {
                    best_idx = last_idx;
                }
            }
        }
        {
            // 4) 状态更新并返回
            last_chosen_armor_id = armors[*best_idx].id;
            return { armors[*best_idx] };
        }
        {
            // 3) 切换抖动抑制
            if (last_idx.has_value() && (*last_idx != *best_idx)) {
                auto const last_abs    = std::abs(candidate_evals[*last_idx].delta_yaw);
                auto const best_abs    = std::abs(candidate_evals[*best_idx].delta_yaw);
                auto const improvement = last_abs - best_abs;
                if (improvement < min_switch_improvement_angle) {
                    best_idx = last_idx;
                }
            }
        }
        {
            // 4) 状态更新并返回
            last_chosen_armor_id = armors[*best_idx].id;
            return { armors[*best_idx] };
        }
    }

private:
    Config config_ {};

    struct CandidateEval {
        double delta_yaw { 0.0 };
        bool in_window { false };
    };

    const double min_switch_improvement_angle { util::deg2rad(7.0) };
    std::optional<int> last_chosen_armor_id {};
};

AimPointChooser::AimPointChooser() noexcept
    : pimpl { std::make_unique<Impl>() } { }

AimPointChooser::~AimPointChooser() noexcept = default;

auto AimPointChooser::initialize(Config const& config) noexcept -> void {
    return pimpl->initialize(config);
}

auto AimPointChooser::choose_armor(std::span<Armor3D const> armors,
    Eigen::Vector3d const& center_position, double angular_velocity) -> std::optional<Armor3D> {
    return pimpl->choose_armor(armors, center_position, angular_velocity);
}

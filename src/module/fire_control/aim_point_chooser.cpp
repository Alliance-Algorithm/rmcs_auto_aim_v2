#include "aim_point_chooser.hpp"
#include "utility/math/conversion.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <tuple>
#include <vector>

using namespace rmcs::fire_control;

struct AimPointChooser::Impl {
    struct CandidateEval {
        double delta_yaw { 0.0 };
        bool in_window { false };
    };

    struct AngleWindow {
        double coming { 0.0 };
        double leaving { 0.0 };
    };

    AngleWindow normal_fast_window { util::deg2rad(70.0), util::deg2rad(20.0) }; // rad
    AngleWindow outpost_window { util::deg2rad(70.0), util::deg2rad(30.0) };     // rad
    const double min_switch_improvement_angle { util::deg2rad(7.0) };
    double angular_velocity_threshold { 2.0 };

    std::optional<int> last_chosen_armor_id { };

    auto initialize(Config const& config) noexcept -> void {
        normal_fast_window = { config.coming_angle, config.leaving_angle };
        outpost_window     = { config.outpost_coming_angle, config.outpost_leaving_angle };
        angular_velocity_threshold = config.angular_velocity_threshold;
    }

    auto choose_armor(std::span<Armor3D const> armors, Eigen::Vector3d const& center_position,
        double angular_velocity) -> std::optional<Armor3D> {
        if (armors.empty()) {
            last_chosen_armor_id.reset();
            return std::nullopt;
        }

        const auto center_yaw = std::atan2(center_position.y(), center_position.x());
        const auto is_outpost = armors.front().genre == DeviceId::OUTPOST;

        auto candidate_evals = std::vector<CandidateEval>(armors.size());

        const auto yaw = [&](size_t index) {
            auto orientation = Eigen::Quaterniond { };
            armors[index].orientation.copy_to(orientation);
            return util::eulers(orientation)[0];
        };

        const auto in_window = [&](double delta_yaw, AngleWindow const& active_window) {
            const auto abs_delta  = std::abs(delta_yaw);
            const auto in_coming  = abs_delta <= active_window.coming;
            const auto in_leaving = (angular_velocity > 0.0) ? (delta_yaw <= active_window.leaving)
                : (angular_velocity < 0.0)                   ? (delta_yaw >= -active_window.leaving)
                                                             : true;
            return in_coming && in_leaving;
        };

        const auto yaw_delta_at = [&](size_t index) {
            return util::normalize_angle(yaw(index) - center_yaw);
        };

        if (!is_outpost && std::abs(angular_velocity) < angular_velocity_threshold) {
            struct SlowCandidate {
                size_t index;
                double delta_yaw;
                double score;
            };

            auto candidates = std::array<SlowCandidate, 4> { };
            const auto count = std::min(armors.size(), candidates.size());

            for (size_t index = 0; index < count; ++index) {
                const auto delta_yaw = yaw_delta_at(index);
                auto score           = std::abs(delta_yaw);

                if (last_chosen_armor_id.has_value() && armors[index].id == *last_chosen_armor_id) {
                    score -= util::deg2rad(8.0);
                }

                candidates[index] = SlowCandidate {
                    .index = index,
                    .delta_yaw = delta_yaw,
                    .score = score,
                };
            }

            const auto best = std::min_element(candidates.begin(), candidates.begin() + count,
                [](SlowCandidate const& lhs, SlowCandidate const& rhs) {
                    const auto lhs_score = (std::abs(lhs.delta_yaw) > util::deg2rad(90.0)) ? 1e5 : lhs.score;
                    const auto rhs_score = (std::abs(rhs.delta_yaw) > util::deg2rad(90.0)) ? 1e5 : rhs.score;
                    return lhs_score < rhs_score;
                });

            if (best != candidates.begin() + count
                && std::abs(best->delta_yaw) < util::deg2rad(90.0)) {
                last_chosen_armor_id = armors[best->index].id;
                return { armors[best->index] };
            }

            last_chosen_armor_id.reset();
            return std::nullopt;
        }

        const auto& active_window = is_outpost ? outpost_window : normal_fast_window;

        { // 1) 候选评估
            for (size_t index = 0; index < armors.size(); ++index) {
                const auto delta_yaw   = yaw_delta_at(index);
                candidate_evals[index] = {
                    .delta_yaw = delta_yaw,
                    .in_window = in_window(delta_yaw, active_window),
                };
            }
        }

        const auto priority_key = [&](size_t index) {
            // 优先级：
            // 1) abs_delta：角误差更小优先
            // 2) last_penalty：上一帧目标优先（is_last -> 0，其它 -> 1）
            // 3) id：稳定排序
            // 4) index：最终兜底，保证结果确定性
            const auto abs_delta = std::abs(candidate_evals[index].delta_yaw);
            const auto id        = armors[index].id;
            const auto is_last = last_chosen_armor_id.has_value() && (id == *last_chosen_armor_id);
            const auto last_penalty = is_last ? 0 : 1;
            return std::tuple { abs_delta, last_penalty, id, index };
        };

        auto best_idx = std::optional<size_t> { };
        auto last_idx = std::optional<size_t> { };

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
                const auto last_abs    = std::abs(candidate_evals[*last_idx].delta_yaw);
                const auto best_abs    = std::abs(candidate_evals[*best_idx].delta_yaw);
                const auto improvement = last_abs - best_abs;
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

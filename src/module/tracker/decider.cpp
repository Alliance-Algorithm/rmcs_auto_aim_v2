#include "decider.hpp"
#include "module/predictor/robot_state.hpp"
#include "utility/time.hpp"

#include <cmath>
#include <limits>
#include <tuple>
#include <unordered_map>
#include <vector>

using namespace rmcs::tracker;
using namespace rmcs::predictor;
using namespace std::chrono_literals;

struct Decider::Impl {
    static constexpr auto kDefaultCleanupInterval = 1.0s;
    static constexpr auto kOutpostCleanupInterval = 1.5s;
    static constexpr auto kLockLostTimeout        = 150ms;

    static constexpr auto get_cleanup_interval(DeviceId device_id) {
        switch (device_id) {
        case DeviceId::OUTPOST:
            return kOutpostCleanupInterval;
        default:
            return kDefaultCleanupInterval;
        }
    }

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        std::ignore = yaml;

        if (priority_mode.empty()) priority_mode = mode2;

        return {};
    }

    auto set_priority_mode(PriorityMode const& mode) -> void { priority_mode = mode; }

    auto set_target_mode(TargetMode mode) -> void { target_mode = mode; }

    auto update(std::span<Armor3D const> armors, TimePoint t) -> Output {
        // 推进所有现有追踪器的时间轴
        for (auto& [id, tracker] : trackers) {
            tracker->predict(t);
        }

        auto fused_ids      = DeviceIds::None();
        auto grouped_armors = std::unordered_map<DeviceId, std::vector<Armor3D>> {};

        for (const auto& armor : armors) {
            grouped_armors[armor.genre].emplace_back(armor);
        }

        for (auto& [id, grouped] : grouped_armors) {
            if (!trackers.contains(id)) {
                trackers[id] = std::make_unique<RobotState>();
                trackers[id]->initialize(grouped.front(), t);
                last_seen_times[id] = t;
            }

            auto grouped_span = std::span<Armor3D const> { grouped.data(), grouped.size() };
            bool fused        = trackers[id]->update(grouped_span);

            if (fused) {
                fused_ids.append(id);
                last_seen_times[id] = t;
            }
        }

        std::erase_if(trackers, [&](const auto& item) {
            auto last_seen_it     = last_seen_times.find(item.first);
            auto cleanup_interval = get_cleanup_interval(item.first);
            bool expired          = last_seen_it == last_seen_times.end()
                || util::delta_time(t, last_seen_it->second) > cleanup_interval;
            if (expired) {
                last_seen_times.erase(item.first);
            }

            return expired;
        });

        if (!target_mode_allows(target_mode, locked_target_id)) {
            locked_target_id = DeviceId::UNKNOWN;
        }

        // 优先保持当前锁定目标的连续性：本帧融合命中则继续控制，
        // 短时失融时继续保留 target_id 但禁止控制。
        if (locked_target_id != DeviceId::UNKNOWN) {
            if (trackers.contains(locked_target_id)) {
                if (fused_ids.contains(locked_target_id)) {
                    return make_output(locked_target_id, true);
                }

                auto locked_seen_it = last_seen_times.find(locked_target_id);
                bool within_grace   = locked_seen_it != last_seen_times.end()
                    && util::delta_time(t, locked_seen_it->second) <= kLockLostTimeout;

                if (within_grace) {
                    return make_output(locked_target_id, false);
                }
            }

            locked_target_id = DeviceId::UNKNOWN;
        }

        auto fresh_target_id = arbitrate(fused_ids);
        if (fresh_target_id != DeviceId::UNKNOWN) {
            // 未命中当前锁定目标时，从本帧成功融合的目标里选出新的锁定目标。
            locked_target_id = fresh_target_id;
            return make_output(fresh_target_id, true);
        }

        return Output {
            .target_id     = DeviceId::UNKNOWN,
            .snapshot      = std::nullopt,
            .allow_control = false,
        };
    }

    auto arbitrate(DeviceIds fused_ids) -> DeviceId {
        auto best_target_id = DeviceId::UNKNOWN;

        for (const auto& [device_id, _] : trackers) {
            if (!fused_ids.contains(device_id)) continue;
            if (!target_mode_allows(target_mode, device_id)) continue;

            if (best_target_id == DeviceId::UNKNOWN
                || is_better_target(device_id, best_target_id)) {
                best_target_id = device_id;
            }
        }

        return best_target_id;
    }

    auto make_output(DeviceId device_id, bool fused) const -> Output {
        return Output {
            .target_id     = device_id,
            .snapshot      = trackers.at(device_id)->get_snapshot(),
            .allow_control = fused && trackers.at(device_id)->is_converged(),
        };
    }

    auto priority_of(DeviceId device_id) const -> int {
        if (auto it = priority_mode.find(device_id); it != priority_mode.end()) {
            return it->second;
        }
        return std::numeric_limits<int>::max();
    }

    auto is_better_target(DeviceId lhs, DeviceId rhs) const -> bool {
        auto rank = [&](DeviceId device_id) {
            auto const& tracker = *trackers.at(device_id);
            auto distance       = tracker.distance();
            auto safe_distance =
                std::isfinite(distance) ? distance : std::numeric_limits<double>::infinity();

            // 比较顺序：距离 -> 收敛状态 -> 优先级 -> 固定 ID 兜底。
            return std::tuple {
                safe_distance,             // 非有限距离按无穷远处理，避免 NaN/Inf 干扰排序。
                !tracker.is_converged(),   // 收敛目标映射为 0，未收敛目标映射为 1。
                priority_of(device_id),    // 数值越小，优先级越高。
                rmcs::to_index(device_id), // 完全相同时按固定顺序兜底，避免容器遍历顺序抖动。
            };
        };

        return rank(lhs) < rank(rhs);
    }

    std::unordered_map<DeviceId, std::unique_ptr<RobotState>> trackers;
    std::unordered_map<DeviceId, TimePoint> last_seen_times;
    DeviceId locked_target_id { DeviceId::UNKNOWN };
    TargetMode target_mode { TargetMode::COMBAT };

    PriorityMode priority_mode;

    const PriorityMode mode1 = {
        { DeviceId::HERO, 2 },
        { DeviceId::ENGINEER, 4 },
        { DeviceId::INFANTRY_3, 1 },
        { DeviceId::INFANTRY_4, 1 },
        { DeviceId::INFANTRY_5, 5 },
        { DeviceId::SENTRY, 3 },
        { DeviceId::OUTPOST, 2 },
        { DeviceId::BASE, 5 },
        { DeviceId::UNKNOWN, 5 },
    };

    const PriorityMode mode2 = {
        { DeviceId::HERO, 1 },
        { DeviceId::ENGINEER, 2 },
        { DeviceId::INFANTRY_3, 1 },
        { DeviceId::INFANTRY_4, 1 },
        { DeviceId::INFANTRY_5, 5 },
        { DeviceId::SENTRY, 3 },
        { DeviceId::OUTPOST, 1 },
        { DeviceId::BASE, 4 },
        { DeviceId::UNKNOWN, 5 },
    };
};

Decider::Decider() noexcept
    : pimpl { std::make_unique<Impl>() } { }
Decider::~Decider() noexcept = default;

auto Decider::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto Decider::set_priority_mode(PriorityMode const& mode) -> void {
    return pimpl->set_priority_mode(mode);
}

auto Decider::set_target_mode(TargetMode mode) -> void { return pimpl->set_target_mode(mode); }

auto Decider::update(std::span<Armor3D const> armors, TimePoint t) -> Output {
    return pimpl->update(armors, t);
}

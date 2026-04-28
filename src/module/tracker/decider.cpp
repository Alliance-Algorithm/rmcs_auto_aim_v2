#include "decider.hpp"
#include "module/predictor/robot_state.hpp"
#include "utility/serializable.hpp"
#include "utility/time.hpp"

#include <cmath>
#include <limits>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace rmcs::tracker;
using namespace rmcs::predictor;
using namespace std::chrono_literals;

struct Decider::Impl {
    static constexpr auto kDefaultCleanupInterval = 1.0s;
    static constexpr auto kOutpostCleanupInterval = 1.5s;
    static constexpr auto kReleaseAfterFrames     = std::size_t { 3 };

    struct TargetMemory {
        std::optional<TimePoint> last_seen_time { };
        std::size_t consecutive_missing_frames { 0 };
        std::size_t consecutive_stable_frames { 0 };
        std::size_t consecutive_instable_frames { 0 };
        bool temporary_lost_armed { false };
    };

    struct Config : util::Serializable {
        std::size_t max_temporary_loss_frames { 5 };
        std::size_t max_unconfirmed_loss_frames { 3 };
        std::size_t tracking_confirm_frames { 3 };

        constexpr static std::tuple metas {
            &Config::max_temporary_loss_frames,
            "max_temporary_loss_frames",
            &Config::max_unconfirmed_loss_frames,
            "max_unconfirmed_loss_frames",
            &Config::tracking_confirm_frames,
            "tracking_confirm_frames",
        };
    };

    static constexpr auto get_cleanup_interval(DeviceId device_id) {
        switch (device_id) {
        case DeviceId::OUTPOST:
            return kOutpostCleanupInterval;
        default:
            return kDefaultCleanupInterval;
        }
    }

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        if (config.max_temporary_loss_frames == 0) {
            return std::unexpected { "tracker.max_temporary_loss_frames must be > 0" };
        }
        if (config.max_unconfirmed_loss_frames == 0) {
            return std::unexpected { "tracker.max_unconfirmed_loss_frames must be > 0" };
        }
        if (config.tracking_confirm_frames == 0) {
            return std::unexpected { "tracker.tracking_confirm_frames must be > 0" };
        }

        if (priority_mode.empty()) priority_mode = mode2;

        return { };
    }

    auto set_priority_mode(PriorityMode const& mode) -> void { priority_mode = mode; }

    auto update(std::span<Armor3D const> armors, TimePoint t) -> Output {
        // 推进所有现有追踪器的时间轴
        for (auto& [id, tracker] : trackers) {
            tracker->predict(t);
        }

        auto observed_ids   = std::unordered_set<DeviceId> { };
        auto grouped_armors = std::unordered_map<DeviceId, std::vector<Armor3D>> { };

        for (const auto& armor : armors) {
            grouped_armors[armor.genre].emplace_back(armor);
        }

        for (auto& [id, grouped] : grouped_armors) {
            auto& target_memory = target_memories[id];
            if (!trackers.contains(id)) {
                trackers[id] = std::make_unique<RobotState>();
                trackers[id]->initialize(grouped.front(), t);
            }

            auto grouped_span = std::span<Armor3D const> { grouped.data(), grouped.size() };
            bool fused        = trackers[id]->update(grouped_span);

            if (fused) {
                observed_ids.insert(id);
                target_memory.last_seen_time             = t;
                target_memory.consecutive_missing_frames = 0;
            }
        }

        // 状态机：
        // 1. unconfirmed: 已有 tracker，但还没稳定到可接管；
        // 2. confirmed: 连续稳定若干帧后允许控制接管；
        // 3. temporary lost: confirmed 目标短暂丢失时，保留控制输出窗口。
        // 4. confirmed → unconfirmed: 需要连续不稳定 kReleaseAfterFrames 帧才释放
        for (const auto& [id, tracker] : trackers) {
            auto& target_memory         = target_memories[id];
            auto was_tracking_confirmed = tracking_confirmed(id);

            if (observed_ids.contains(id) && tracker->is_converged()) {
                ++target_memory.consecutive_stable_frames;
                target_memory.consecutive_instable_frames = 0;
                target_memory.temporary_lost_armed        = false;
                continue;
            }

            if (observed_ids.contains(id)) {
                ++target_memory.consecutive_instable_frames;
                if (target_memory.consecutive_instable_frames < kReleaseAfterFrames) {
                    continue;
                }
            }

            target_memory.consecutive_stable_frames = 0;
            if (!observed_ids.contains(id)) {
                if (was_tracking_confirmed) {
                    target_memory.temporary_lost_armed = true;
                }
                ++target_memory.consecutive_missing_frames;
            } else {
                target_memory.temporary_lost_armed = false;
            }
        }

        std::erase_if(trackers, [&](const auto& item) {
            auto memory_it        = target_memories.find(item.first);
            auto cleanup_interval = get_cleanup_interval(item.first);
            bool expired = memory_it == target_memories.end() || !memory_it->second.last_seen_time
                || util::delta_time(t, *memory_it->second.last_seen_time) > cleanup_interval;
            if (expired) {
                if (item.first == primary_target_id) primary_target_id = DeviceId::UNKNOWN;
                target_memories.erase(item.first);
            }

            return expired;
        });

        auto fresh_target_id = arbitrate(observed_ids);
        if (fresh_target_id != DeviceId::UNKNOWN) {
            primary_target_id = fresh_target_id;

            auto confirmed = tracking_confirmed(primary_target_id);
            return make_output(primary_target_id, confirmed, confirmed);
        }

        if (auto output = hold_output(primary_target_id)) {
            return std::move(*output);
        }

        primary_target_id = DeviceId::UNKNOWN;
        return Output {
            .target_id          = DeviceId::UNKNOWN,
            .snapshot           = std::nullopt,
            .allow_control      = false,
            .tracking_confirmed = false,
        };
    }

    auto arbitrate(const std::unordered_set<DeviceId>& observed_ids) -> DeviceId {
        auto best_target_id = DeviceId::UNKNOWN;

        for (const auto& [device_id, _] : trackers) {
            if (!observed_ids.contains(device_id)) continue;

            if (best_target_id == DeviceId::UNKNOWN
                || is_better_target(device_id, best_target_id)) {
                best_target_id = device_id;
            }
        }

        return best_target_id;
    }

    auto tracking_confirmed(DeviceId device_id) const -> bool {
        auto memory_it = target_memories.find(device_id);
        if (memory_it == target_memories.end()) {
            return false;
        }

        return memory_it->second.consecutive_stable_frames >= config.tracking_confirm_frames;
    }

    auto make_output(DeviceId device_id, bool allow_takeover, bool confirmed) const -> Output {
        return Output {
            .target_id          = device_id,
            .snapshot           = trackers.at(device_id)->get_snapshot(),
            .allow_control      = allow_takeover,
            .tracking_confirmed = confirmed,
        };
    }

    auto hold_output(DeviceId device_id) const -> std::optional<Output> {
        if (device_id == DeviceId::UNKNOWN || !trackers.contains(device_id)) {
            return std::nullopt;
        }

        const auto& target_tracker = *trackers.at(device_id);
        const auto& target_memory  = target_memories.at(device_id);

        if (target_tracker.is_converged()) {
            if (target_memory.temporary_lost_armed
                && is_within_loss_window(device_id, config.max_temporary_loss_frames)) {
                return make_output(device_id, true, false);
            }
            return std::nullopt;
        }

        if (is_within_loss_window(device_id, config.max_unconfirmed_loss_frames)) {
            return make_output(device_id, false, false);
        }

        return std::nullopt;
    }

    auto is_within_loss_window(DeviceId device_id, std::size_t max_missing_frames) const -> bool {
        auto memory_it = target_memories.find(device_id);
        if (device_id == DeviceId::UNKNOWN || !trackers.contains(device_id)
            || memory_it == target_memories.end() || !memory_it->second.last_seen_time) {
            return false;
        }

        return memory_it->second.consecutive_missing_frames <= max_missing_frames;
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

            // 比较顺序：优先级 -> 收敛状态 -> 距离 -> 固定 ID 兜底。
            return std::tuple {
                priority_of(device_id),    // 数值越小，优先级越高。
                !tracker.is_converged(),   // 收敛目标映射为 0，未收敛目标映射为 1。
                safe_distance,             // 非有限距离按无穷远处理，避免 NaN/Inf 干扰排序。
                rmcs::to_index(device_id), // 完全相同时按固定顺序兜底，避免容器遍历顺序抖动。
            };
        };

        return rank(lhs) < rank(rhs);
    }

    DeviceId primary_target_id { DeviceId::UNKNOWN };
    std::unordered_map<DeviceId, std::unique_ptr<RobotState>> trackers;
    std::unordered_map<DeviceId, TargetMemory> target_memories;

    Config config { };
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
        { DeviceId::BASE, 5 },
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

auto Decider::update(std::span<Armor3D const> armors, TimePoint t) -> Output {
    return pimpl->update(armors, t);
}

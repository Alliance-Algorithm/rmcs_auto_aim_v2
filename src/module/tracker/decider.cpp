#include "decider.hpp"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include "module/predictor/robot_state.hpp"
#include "utility/serializable.hpp"
#include "utility/time.hpp"

using namespace rmcs::tracker;
using namespace rmcs::predictor;
using namespace std::chrono_literals;

struct Decider::Impl {
    static constexpr auto kDefaultCleanupInterval = 500ms;

    static constexpr double kPriorityScoreBase = 10.0;
    static constexpr double kDistanceScoreWeight = 5.0;
    static constexpr double kDistanceScoreBias = 1.0;
    static constexpr double kConvergedScoreBonus = 4.0;
    static constexpr double kPrimaryTargetScoreBonus = 2.0;

    struct TargetMemory {
        std::optional<Clock::time_point> last_seen_time {};
        std::size_t consecutive_missing_frames { 0 };
        std::size_t consecutive_stable_frames { 0 };
        bool temporary_lost_armed { false };
    };

    struct Config : util::Serializable {
        std::size_t max_temporary_loss_frames { 5 };
        std::size_t max_detecting_loss_frames { 3 };
        std::size_t tracking_confirm_frames { 3 };

        constexpr static std::tuple metas {
            &Config::max_temporary_loss_frames,
            "max_temporary_loss_frames",
            &Config::max_detecting_loss_frames,
            "max_detecting_loss_frames",
            &Config::tracking_confirm_frames,
            "tracking_confirm_frames",
        };
    };

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        if (config.max_temporary_loss_frames == 0) {
            return std::unexpected { "tracker.max_temporary_loss_frames must be > 0" };
        }
        if (config.max_detecting_loss_frames == 0) {
            return std::unexpected { "tracker.max_detecting_loss_frames must be > 0" };
        }
        if (config.tracking_confirm_frames == 0) {
            return std::unexpected { "tracker.tracking_confirm_frames must be > 0" };
        }

        return {};
    }

    auto set_priority_mode(PriorityMode const& mode) -> void { priority_mode = mode; }

    auto update(std::span<Armor3D const> armors, Clock::time_point t) -> Output {
        // 推进所有现有追踪器的时间轴
        for (auto& [id, tracker] : trackers) {
            tracker->predict(t);
        }

        auto observed_ids = std::unordered_set<DeviceId> {};

        // 将检测到的装甲板按 DeviceId 分发给对应的 RobotState
        for (const auto& armor : armors) {
            auto id = armor.genre;
            auto& target_memory = target_memories[id];

            // 发现新 ID，创建新的追踪器
            if (!trackers.contains(id)) {
                trackers[id] = std::make_unique<RobotState>();
                trackers[id]->initialize(armor, t);
            }

            // 只有融合成功的观测才用于更新可见性和丢失计数。
            auto fused = trackers[id]->update(armor);
            if (fused) {
                observed_ids.insert(id);
                target_memory.last_seen_time          = t;
                target_memory.consecutive_missing_frames = 0;
            }
        }

        for (const auto& [id, tracker] : trackers) {
            auto& target_memory = target_memories[id];
            auto was_tracking_confirmed = tracking_confirmed(id);

            if (observed_ids.contains(id) && tracker->is_converged()) {
                ++target_memory.consecutive_stable_frames;
                target_memory.temporary_lost_armed = false;
                continue;
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
            auto memory_it = target_memories.find(item.first);
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
            return *output;
        }

        primary_target_id = DeviceId::UNKNOWN;
        return Output {
            .target_id          = DeviceId::UNKNOWN,
            .snapshot           = std::nullopt,
            .allow_takeover     = false,
            .tracking_confirmed = false,
        };
    }

    auto arbitrate(const std::unordered_set<DeviceId>& observed_ids) -> DeviceId {
        auto candidates = trackers | std::views::filter([&](const auto& pair) {
            return observed_ids.contains(pair.first);
        });

        if (std::ranges::empty(candidates)) return DeviceId::UNKNOWN;

        auto it = std::ranges::max_element(candidates, {},
            [&](const auto& pair) { return calculate_score(pair.first, *pair.second); });

        return it->first;
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
            .allow_takeover     = allow_takeover,
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

        if (is_within_loss_window(device_id, config.max_detecting_loss_frames)) {
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

    // TODO:需要进一步确定
    //  评分函数：结合优先级模式、距离、收敛情况
    auto calculate_score(DeviceId device, RobotState const& tracker) const -> double {
        double score = 0.0;

        // 基础优先级评分
        if (priority_mode.contains(device)) {
            // RobotPriority 枚举值越小，优先级越高
            score += (kPriorityScoreBase - static_cast<double>(priority_mode.at(device)));
        }

        // 距离加权：优先锁定近处的目标 (简单的 1/dist)
        double dist = tracker.distance();
        score += kDistanceScoreWeight / (dist + kDistanceScoreBias);

        // 优先延续已经收敛的目标，避免频繁切到未收敛目标导致停留 Detecting。
        if (tracker.is_converged()) score += kConvergedScoreBonus;

        // 粘滞性：如果已经是主目标，额外加分防止“摇头”
        if (device == primary_target_id) score += kPrimaryTargetScoreBonus;

        return score;
    }

    DeviceId primary_target_id { DeviceId::UNKNOWN };
    std::unordered_map<DeviceId, std::unique_ptr<RobotState>> trackers;
    std::unordered_map<DeviceId, TargetMemory> target_memories;

    Config config {};
    PriorityMode priority_mode;

    std::chrono::duration<double> cleanup_interval { kDefaultCleanupInterval };

    const PriorityMode mode1 = {
        { DeviceId::HERO, 2 },
        { DeviceId::ENGINEER, 4 },
        { DeviceId::INFANTRY_3, 1 },
        { DeviceId::INFANTRY_4, 1 },
        { DeviceId::INFANTRY_5, 3 },
        { DeviceId::SENTRY, 3 },
        { DeviceId::OUTPOST, 5 },
        { DeviceId::BASE, 5 },
        { DeviceId::UNKNOWN, 5 },
    };

    const PriorityMode mode2 = {
        { DeviceId::HERO, 1 },
        { DeviceId::ENGINEER, 2 },
        { DeviceId::INFANTRY_3, 1 },
        { DeviceId::INFANTRY_4, 2 },
        { DeviceId::INFANTRY_5, 3 },
        { DeviceId::SENTRY, 3 },
        { DeviceId::OUTPOST, 5 },
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

auto Decider::update(std::span<Armor3D const> armors, Clock::time_point t) -> Output {
    return pimpl->update(armors, t);
}

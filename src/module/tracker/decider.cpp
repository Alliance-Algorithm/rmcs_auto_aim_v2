#include "decider.hpp"

#include "module/predictor/robot_state.hpp"
#include "utility/time.hpp"

using namespace rmcs::tracker;
using namespace rmcs::predictor;
using namespace std::chrono_literals;

struct Decider::Impl {
    auto set_priority_mode(PriorityMode const& mode) -> void { priority_mode = mode; }

    auto update(std::span<Armor3D const> armors, Clock::time_point t) -> Output {
        // 推进所有现有追踪器的时间轴
        for (auto& [id, tracker] : trackers) {
            tracker->predict(t);
        }

        // 将检测到的装甲板按 DeviceId 分发给对应的 RobotState
        for (const auto& armor : armors) {
            auto id = armor.genre;

            // 发现新 ID，创建新的追踪器
            if (!trackers.contains(id)) {
                trackers[id] = std::make_unique<RobotState>();
                trackers[id]->initialize(armor, t);
            }

            // RobotState 内部会调用 match() 自动处理多装甲板逻辑
            trackers[id]->update(armor);
            last_seen_time[id] = t;
        }

        std::erase_if(trackers, [&](const auto& item) {
            bool expired = util::delta_time(t, last_seen_time[item.first]) > cleanup_interval;
            if (expired) {
                if (item.first == primary_target_id) primary_target_id = DeviceId::UNKNOWN;
                last_seen_time.erase(item.first);
            }

            return expired;
        });

        primary_target_id = arbitrate(t);

        if (primary_target_id != DeviceId::UNKNOWN) {
            auto& target_tracker = trackers[primary_target_id];
            return {
                .state     = target_tracker->is_converged() ? State::Tracking : State::Detecting,
                .target_id = primary_target_id,
                .snapshot  = target_tracker->get_snapshot(),
            };
        }
        return {
            .state     = State::Lost,
            .target_id = DeviceId::UNKNOWN,
            .snapshot  = std::nullopt,
        };
    }

    auto arbitrate(Clock::time_point now) -> DeviceId {
        auto candidates = trackers | std::views::filter([&](const auto& pair) {
            return util::delta_time(now, last_seen_time[pair.first]) < active_interval;
        });

        if (std::ranges::empty(candidates)) return DeviceId::UNKNOWN;

        auto it = std::ranges::max_element(candidates, {},
            [&](const auto& pair) { return calculate_score(pair.first, *pair.second); });

        return it->first;
    }

    // TODO:需要进一步确定
    //  评分函数：结合优先级模式、距离、收敛情况
    auto calculate_score(DeviceId device, RobotState const& tracker) const -> double {
        double score = 0.0;

        // 基础优先级评分
        if (priority_mode.contains(device)) {
            // RobotPriority 枚举值越小，优先级越高
            score += (10.0 - static_cast<double>(priority_mode.at(device)));
        }

        // 距离加权：优先锁定近处的目标 (简单的 1/dist)
        double dist = tracker.distance();
        score += 5.0 / (dist + 1.0);

        // 粘滞性：如果已经是主目标，额外加分防止“摇头”
        if (device == primary_target_id) score += 2.0;

        return score;
    }

    DeviceId primary_target_id { DeviceId::UNKNOWN };
    std::unordered_map<DeviceId, std::unique_ptr<RobotState>> trackers;
    std::unordered_map<DeviceId, Clock::time_point> last_seen_time;

    PriorityMode priority_mode;

    std::chrono::duration<double> cleanup_interval { 500ms };
    std::chrono::duration<double> active_interval { 100ms };

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

auto Decider::set_priority_mode(PriorityMode const& mode) -> void {
    return pimpl->set_priority_mode(mode);
}

auto Decider::update(std::span<Armor3D const> armors, Clock::time_point t) -> Output {
    return pimpl->update(armors, t);
}

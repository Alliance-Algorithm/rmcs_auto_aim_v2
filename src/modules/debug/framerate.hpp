#pragma once
#include <chrono>
#include <deque>

namespace rmcs {

class FramerateCounter {
public:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    std::deque<TimePoint> frame_times;

    TimePoint last_tick_timestamp;
    std::chrono::milliseconds interval = std::chrono::seconds { 2 };

    auto tick() noexcept -> bool {
        using namespace std::chrono_literals;

        const auto now = Clock::now();
        frame_times.push_back(now);

        while (!frame_times.empty() && now - frame_times.front() > 1s) {
            frame_times.pop_front();
        }

        auto is_reach_interval = bool { false };
        if (interval.count() > 0 && now - last_tick_timestamp > interval) {
            is_reach_interval   = true;
            last_tick_timestamp = now;
        }

        return is_reach_interval;
    }

    auto fps() const { return frame_times.size(); }

    auto set_intetval(std::chrono::milliseconds ms) { interval = ms; }
};

}

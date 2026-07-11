#pragma once
#include <string>
#include <utility>
#include <vector>

#include "utility/clock.hpp"

namespace rmcs {

class Repeat {
public:
    struct Action {
        std::string tag;
        Duration duration;

        explicit Action(std::string tag, Duration duration)
            : tag { std::move(tag) }
            , duration { duration } { }

        explicit Action(std::string tag, double seconds)
            : tag { std::move(tag) }
            , duration { std::chrono::duration_cast<Duration>(
                  std::chrono::duration<double> { seconds }) } { }
    };
    static auto make(std::string tag, Duration duration) noexcept {
        return Action { std::move(tag), duration };
    }

    Repeat() = default;

    template <typename... Args>
        requires(std::same_as<std::decay_t<Args>, Action> && ...)
    explicit Repeat(Args&&... actions) {
        (actions_.push_back(std::forward<Args>(actions)), ...);
    }

    auto push(Action action) noexcept { actions_.push_back(std::move(action)); }

    auto clear() noexcept { actions_.clear(); }

    auto tick() -> std::string_view {
        if (!start_ || actions_.empty()) {
            return { };
        }

        const auto now      = Clock::now();
        const auto duration = now - timestamp_start_;

        if (duration > next_duration_) {
            progress_ = (progress_ + 1) % actions_.size();

            const auto& action = actions_[progress_];
            if (progress_ == 0) {
                next_duration_   = action.duration;
                timestamp_start_ = now;
            } else {
                next_duration_ = action.duration + next_duration_;
            }

            return action.tag;
        }

        return actions_[progress_].tag;
    }

    auto start() noexcept {
        timestamp_start_ = Clock::now();
        progress_        = 0;
        next_duration_   = actions_.empty() ? Duration::zero() : actions_.front().duration;

        start_ = true;
    }
    auto started() const noexcept { return start_; }

    auto stop() noexcept { start_ = false; }

private:
    Timestamp timestamp_start_;
    bool start_ = false;

    std::size_t progress_ = 0;
    Duration next_duration_;
    std::vector<Action> actions_;
};

}

#pragma once

#include <chrono>
#include <concepts>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

#include "module/debug/framerate.hpp"
#include "utility/times_limit.hpp"

namespace rmcs::util {

class ActionThrottler {
public:
    using duration = std::chrono::milliseconds;

    ActionThrottler(duration interval, std::size_t default_quota) noexcept
        : interval_ { interval }
        , default_quota_ { default_quota } { }

    auto register_action(std::string_view tag, std::optional<std::size_t> quota = std::nullopt)
        -> void {
        auto [it, inserted] = actions_.try_emplace(
            std::string { tag }, Action { interval_, quota.value_or(default_quota_) });
        if (!inserted) {
            it->second = Action { interval_, quota.value_or(default_quota_) };
        }
    }

    template <std::invocable Fn>
    auto dispatch(std::string_view tag, Fn&& action) -> bool {
        auto it = actions_.find(tag);
        if (it == actions_.end()) return false;

        auto& action_state = it->second;
        if (!action_state.metronome.tick()) return false;

        if (action_state.limit.tick()) {
            std::forward<Fn>(action)();
            return true;
        }

        action_state.limit.disable();
        return false;
    }

    auto reset(std::string_view tag) -> void {
        if (auto it = actions_.find(tag); it != actions_.end()) {
            it->second.reset();
            it->second.enable();
        }
    }

private:
    struct Action {
        explicit Action(duration interval, std::size_t quota) noexcept
            : limit { quota } {
            metronome.set_interval(interval);
        }

        TimesLimit limit;
        FramerateCounter metronome;

        auto reset() noexcept -> void {
            limit.reset();
            metronome.last_reach_interval_timestamp = {};
        }

        auto enable() noexcept -> void { limit.enable(); }
    };

    struct string_hash {
        using is_transparent = void;
        auto operator()(std::string_view sv) const -> std::size_t {
            return std::hash<std::string_view> {}(sv);
        }
    };

    duration interval_;
    std::size_t default_quota_;
    std::unordered_map<std::string, Action, string_hash, std::equal_to<>> actions_;
};

} // namespace rmcs::util

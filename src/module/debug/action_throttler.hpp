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
    struct string_hash {
        using is_transparent = void;
        auto operator()(std::string_view sv) const -> std::size_t {
            return std::hash<std::string_view> {}(sv);
        }
    };

public:
    using duration = std::chrono::milliseconds;

    ActionThrottler(duration interval, std::size_t default_quota) noexcept
        : default_quota_ { default_quota } {
        metronome_.set_interval(interval);
    }

    auto register_action(std::string_view tag, std::optional<std::size_t> quota = std::nullopt)
        -> void {
        auto [it, inserted] =
            actions_.try_emplace(std::string { tag }, quota.value_or(default_quota_));
        if (!inserted) {
            it->second.limit = quota.value_or(default_quota_);
            it->second.reset();
            it->second.enable();
        }
    }

    template <std::invocable Fn>
    auto dispatch(std::string_view tag, Fn&& action) -> bool {
        if (!metronome_.tick()) return false;

        auto it = actions_.find(tag);
        if (it == actions_.end()) return false;

        auto& limit = it->second;
        if (limit.tick()) {
            std::forward<Fn>(action)();
            return true;
        }

        limit.disable();
        return false;
    }

    auto reset(std::string_view tag) -> void {
        if (auto it = actions_.find(tag); it != actions_.end()) {
            it->second.reset();
            it->second.enable();
        }
    }

private:
    ::rmcs::FramerateCounter metronome_;
    std::size_t default_quota_;
    std::unordered_map<std::string, TimesLimit, string_hash, std::equal_to<>> actions_;
};

} // namespace rmcs::util

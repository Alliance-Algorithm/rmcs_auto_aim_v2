#pragma once

#include <string>
#include <unordered_map>

#include "utility/times_limit.hpp"

namespace rmcs::util {
class LogLimiter {
    std::unordered_map<std::string, TimesLimit> limiters_;
    std::size_t default_limit_;

public:
    explicit LogLimiter(std::size_t default_limit)
        : default_limit_(default_limit) { }

    auto register_key(std::string_view key) -> void {
        limiters_.emplace(std::string(key), default_limit_);
    }

    auto tick(std::string_view key) -> bool {
        auto it = limiters_.find(std::string(key));
        if (it == limiters_.end()) return false;

        auto ok = it->second.tick();
        if (!ok) it->second.disable();

        return ok;
    }

    auto enabled(std::string_view key) -> bool {
        auto it = limiters_.find(std::string(key));
        return it != limiters_.end() ? it->second.enabled() : false;
    }

    auto reset(std::string_view key) -> void {
        if (auto it = limiters_.find(std::string(key)); it != limiters_.end()) {
            it->second.reset();
            it->second.enable();
        }
    }
};

}

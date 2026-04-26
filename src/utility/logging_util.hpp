#pragma once

#include "utility/rclcpp/node.hpp"

#include <concepts>
#include <cstdint>
#include <format>
#include <string_view>
#include <unordered_map>
#include <utility>

namespace rmcs::util {

class LoggingUtil {
private:
    RclcppNode& rclcpp;
    std::unordered_map<std::string_view, std::int16_t> store;

    template <std::invocable Fn>
    auto exec(std::string_view name, Fn&& f) {
        if (!store.contains(name)) return;

        if (auto& limit = store.at(name); limit > 0) {
            limit--;
            std::forward<Fn>(f)();
        }
    }

public:
    explicit LoggingUtil(RclcppNode& rclcpp) noexcept
        : rclcpp { rclcpp } { }

    auto reset(std::string_view name, std::uint8_t limit) { store[name] = limit; }

    template <typename... Args>
    auto info(std::string_view name, std::format_string<Args...> fmt, Args&&... args) noexcept {
        exec(name, [=, this] { rclcpp.info(fmt, std::forward<Args>(args)...); });
    }

    template <typename... Args>
    auto warn(std::string_view name, std::format_string<Args...> fmt, Args&&... args) noexcept {
        exec(name, [=, this] { rclcpp.warn(fmt, std::forward<Args>(args)...); });
    }

    template <typename... Args>
    auto error(std::string_view name, std::format_string<Args...> fmt, Args&&... args) noexcept {
        exec(name, [=, this] { rclcpp.error(fmt, std::forward<Args>(args)...); });
    }
};

} // namespace rmcs::util

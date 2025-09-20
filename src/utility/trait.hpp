#pragma once

#include <concepts>
#include <string_view>

namespace rmcs {

enum class LoggingLevel { INFO, WARN, ERROR };

template <typename F>
concept logging_trait = std::invocable<F, LoggingLevel, const std::string_view&>;

}

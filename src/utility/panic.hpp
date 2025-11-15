#pragma once
#include <source_location>
#include <string>

namespace rmcs::util {

[[noreturn]] auto panic(const std::string& message,
    const std::source_location& loc = std::source_location::current()) -> void;

}

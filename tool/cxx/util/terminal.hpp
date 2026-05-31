#pragma once

#include <chrono>
#include <optional>
#include <termios.h>

namespace rmcs::tool::util {

class TerminalRawMode {
public:
    TerminalRawMode();
    ~TerminalRawMode();

    TerminalRawMode(const TerminalRawMode&)            = delete;
    TerminalRawMode& operator=(const TerminalRawMode&) = delete;

private:
    bool enabled_ = false;
    ::termios old_ { };
};

auto poll_key(std::chrono::milliseconds timeout) -> std::optional<char>;

} // namespace rmcs::tool::util

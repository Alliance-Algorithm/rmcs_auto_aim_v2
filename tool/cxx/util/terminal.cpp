#include "terminal.hpp"

#include <sys/select.h>
#include <unistd.h>

namespace rmcs::tool::util {

TerminalRawMode::TerminalRawMode() {
    if (tcgetattr(STDIN_FILENO, &old_) != 0) return;

    auto raw = old_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    enabled_        = (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0);
}

TerminalRawMode::~TerminalRawMode() {
    if (enabled_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_);
    }
}

auto poll_key(std::chrono::milliseconds timeout) -> std::optional<char> {
    auto readfds = fd_set { };
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    auto tv = timeval {
        .tv_sec  = static_cast<long>(timeout.count() / 1'000),
        .tv_usec = static_cast<long>((timeout.count() % 1'000) * 1'000),
    };

    if (select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv) <= 0) {
        return std::nullopt;
    }

    char key = 0;
    if (read(STDIN_FILENO, &key, 1) != 1) {
        return std::nullopt;
    }

    return key;
}

} // namespace rmcs::tool::util

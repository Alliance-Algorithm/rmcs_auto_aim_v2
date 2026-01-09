#pragma once

#include <string>

namespace rmcs::tracker {
enum class State {
    Lost,
    Detecting,
    Tracking,
    TemporaryLost,
    Switching,
};

constexpr auto to_string(State state) -> std::string {
    switch (state) {
    case State::Lost:
        return "Lost";
    case State::Detecting:
        return "Detecting";
    case State::Tracking:
        return "Tracking";
    case State::TemporaryLost:
        return "TemporaryLost";
    case State::Switching:
        return "Switching";
    }
    return "Unknown";
};
}

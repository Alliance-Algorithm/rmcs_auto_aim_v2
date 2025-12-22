#pragma once

<<<<<<< HEAD
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
=======
#include "utility/pimpl.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::tracker {
struct StateMachine {
    RMCS_PIMPL_DEFINITION(StateMachine)
public:
    enum class State {
        Lost,          //
        Detecting,     //
        Tracking,      //
        TemporaryLost, //
        Switching      //
    };

    auto update(bool found, DeviceId found_device) -> void;
>>>>>>> 0a54c02 (wip(decision): implement priority-based armor plate selection (partially complete))
};
}

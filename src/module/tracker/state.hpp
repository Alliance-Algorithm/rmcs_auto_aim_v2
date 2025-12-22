#pragma once

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

    static auto to_string(State state) -> std::string;
};
}

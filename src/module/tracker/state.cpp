#include "state.hpp"
#include "utility/robot/id.hpp"

#include <chrono>

using namespace rmcs::tracker;
using Clock = std::chrono::steady_clock;
using Stamp = Clock::time_point;

struct StateMachine::Impl {

    auto check_camera_offline(Stamp const& time_point) -> bool {
        if ((time_point - update_time_stamp) > timeout) {
            set_state(State::Lost);
            detect_count      = 0;
            temp_lost_count   = 0;
            update_time_stamp = time_point;
            return true;
        }
        update_time_stamp = time_point;
        return false;
    }

    auto update(Stamp const& now, bool found, DeviceId found_device) -> void {
        if (check_camera_offline(now)) return;

        switch (state) {
        case State::Lost: {
            if (found) {
                set_state(State::Detecting);
                detect_count = 1;
            }
            break;
        }

        case State::Detecting: {
            if (found) {
                detect_count++;
                if (detect_count >= min_detect_count) set_state(State::Tracking);
            } else {
                detect_count = 0;
                set_state((pre_state == State::Switching) ? State::Switching : State::Lost);
            }
            break;
        }

        case State::Tracking: {
            if (!found) {
                temp_lost_count = 1;
                set_state(State::TemporaryLost);
            }
            break;
        }

        case State::Switching: {
            if (found) {
                set_state(State::Detecting);
                temp_lost_count = 0;
            } else {
                temp_lost_count++;
                if (temp_lost_count > max_switch_count) {
                    set_state(State::Lost);
                };
            }
            break;
        }

        case State::TemporaryLost: {
            if (found) {
                temp_lost_count = 0;
                set_state(State::Tracking);
            } else {
                temp_lost_count++;
                const auto max_allow = (found_device == DeviceId::OUTPOST)
                    ? outpost_max_temp_lost_count
                    : normal_max_temp_lost_count;
                if (temp_lost_count > max_allow) {
                    set_state(State::Lost);
                };
            }
            break;
        }
        }
    }

    Stamp update_time_stamp { Clock::now() };

    State state { State::Lost };
    State pre_state { State::Lost };

    int detect_count { 0 };
    int temp_lost_count { 0 };

    const int min_detect_count { 5 };
    const int outpost_max_temp_lost_count { 75 };
    const int normal_max_temp_lost_count { 15 };
    const int max_switch_count { 200 };
    const std::chrono::milliseconds timeout { std::chrono::milliseconds(100) };

    auto set_state(State const& new_state) -> void {
        pre_state = state;
        state     = new_state;
    }
};

StateMachine::StateMachine() noexcept
    : pimpl(std::make_unique<Impl>()) { }
StateMachine::~StateMachine() noexcept = default;

auto StateMachine::update(bool found, DeviceId found_device) -> void {
    pimpl->update(Clock::now(), found, found_device);
}

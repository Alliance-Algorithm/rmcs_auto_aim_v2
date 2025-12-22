#include "decider.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/priority.hpp"

using namespace rmcs::tracker;

struct Decider::Impl {

    auto set_mode(PriorityMode const& priority_mode) -> void { mode = priority_mode; }

    PriorityMode mode;

    const PriorityMode mode1 = {
        { DeviceId::HERO, RobotPriority::SECOND },
        { DeviceId::ENGINEER, RobotPriority::FORTH },
        { DeviceId::INFANTRY_3, RobotPriority::FIRST },
        { DeviceId::INFANTRY_4, RobotPriority::FIRST },
        { DeviceId::INFANTRY_5, RobotPriority::THIRD },
        { DeviceId::SENTRY, RobotPriority::THIRD },
        { DeviceId::OUTPOST, RobotPriority::FIFTH },
        { DeviceId::BASE, RobotPriority::FIFTH },
        { DeviceId::UNKNOWN, RobotPriority::FIFTH },
    };

    const PriorityMode mode2 = {
        { DeviceId::HERO, RobotPriority::FIRST },
        { DeviceId::ENGINEER, RobotPriority::SECOND },
        { DeviceId::INFANTRY_3, RobotPriority::FIRST },
        { DeviceId::INFANTRY_4, RobotPriority::SECOND },
        { DeviceId::INFANTRY_5, RobotPriority::THIRD },
        { DeviceId::SENTRY, RobotPriority::THIRD },
        { DeviceId::OUTPOST, RobotPriority::FIFTH },
        { DeviceId::BASE, RobotPriority::FIFTH },
        { DeviceId::UNKNOWN, RobotPriority::FIFTH },
    };
};

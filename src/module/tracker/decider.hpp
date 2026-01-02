#pragma once

#include "module/predictor/snapshot.hpp"
#include "state.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/id.hpp"
#include "utility/robot/priority.hpp"

namespace rmcs::tracker {

struct Decider {
    RMCS_PIMPL_DEFINITION(Decider)

public:
    struct Output {
        State state;
        DeviceId target_id;
        std::optional<predictor::Snapshot> snapshot;
    };

    auto set_priority_mode(PriorityMode const& mode) -> void;

    auto update(std::span<Armor3D const> armors, std::chrono::steady_clock::time_point t) -> Output;
};
}

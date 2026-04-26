#pragma once

#include <expected>
#include <optional>
#include <span>

#include <yaml-cpp/yaml.h>

#include "module/predictor/snapshot.hpp"
#include "utility/clock.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/id.hpp"
#include "utility/robot/priority.hpp"

namespace rmcs::tracker {

struct Decider {
    RMCS_PIMPL_DEFINITION(Decider)

public:
    struct Output {
        DeviceId target_id;
        std::optional<predictor::Snapshot> snapshot;
        bool allow_takeover { false };
        bool tracking_confirmed { false };
    };

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string>;

    auto set_priority_mode(PriorityMode const& mode) -> void;

    auto update(std::span<Armor3D const> armors, TimePoint t) -> Output;
};
}

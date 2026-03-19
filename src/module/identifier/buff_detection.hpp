#pragma once

#include "utility/image/image.hpp"
#include "utility/pimpl.hpp"

#include "utility/shared/context.hpp"
#include "vc/dataio/dataio.h"
#include "vc/feature/rune_tracker.h"

#include <opencv2/core/types.hpp>
#include <opencv2/core/utility.hpp>
#include <yaml-cpp/yaml.h>

#include <expected>
#include <memory>

namespace rmcs::identifier {

class BuffDetection {
    RMCS_PIMPL_DEFINITION(BuffDetection)

public:
    struct BuffDetectionFrame {
        const std::unique_ptr<rmcs::Image>& image;
        const GyroData& gyro_data;
        const util::ControlState& control_state;
    };

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;
    auto auto_detect(const BuffDetectionFrame&) noexcept -> std::optional<std::shared_ptr<RuneTracker>>;
};
}

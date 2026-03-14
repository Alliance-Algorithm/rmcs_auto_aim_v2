#pragma once

#include "utility/image/image.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

#include <expected>

#include <opencv2/core/types.hpp>
#include <yaml-cpp/yaml.h>

namespace rmcs::identifier {

class BuffDetection {
    RMCS_PIMPL_DEFINITION(BuffDetection)

public:
    struct BuffDetectionFrame;
    struct BuffDetectionResult;
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;
    auto auto_detect(const BuffDetectionFrame&) noexcept -> std::optional<BuffDetectionResult>;
};

}

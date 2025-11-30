#pragma once
#include "utility/math/linear.hpp"
#include "utility/pimpl.hpp"
#include <expected>
#include <yaml-cpp/yaml.h>

namespace rmcs::kernel {

class PoseEstimator {
    RMCS_PIMPL_DEFINITION(PoseEstimator)

public:
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto update_imu_link(const Orientation&) noexcept -> void;
};

}

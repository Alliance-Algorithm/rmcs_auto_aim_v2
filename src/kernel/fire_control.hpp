#pragma once

#include <eigen3/Eigen/Geometry>
#include <expected>
#include <yaml-cpp/yaml.h>

#include "utility/pimpl.hpp"

namespace rmcs::kernel {

class FireControl {
    RMCS_PIMPL_DEFINITION(FireControl)

public:
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;
};
}

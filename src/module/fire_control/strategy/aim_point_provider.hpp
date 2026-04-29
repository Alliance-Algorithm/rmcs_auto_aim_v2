#pragma once

#include <expected>
#include <optional>
#include <string>

#include <yaml-cpp/yaml.h>

#include "module/predictor/snapshot.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::fire_control {

class AimPointProvider {
    RMCS_PIMPL_DEFINITION(AimPointProvider)

public:
    enum class Mode : bool {
        ARMOR,
        CENTER,
    };

    auto configure_yaml(const YAML::Node& yaml) noexcept -> std::expected<void, std::string>;
    auto aim_point_at(predictor::Snapshot const& snapshot, TimePoint t, Mode mode)
        -> std::optional<Eigen::Vector3d>;
};

} // namespace rmcs::fire_control

#pragma once

<<<<<<< HEAD
#include <expected>
#include <yaml-cpp/yaml.h>

#include "module/predictor/snapshot.hpp"
#include "utility/clock.hpp"
#include "utility/math/linear.hpp"
#include "utility/pimpl.hpp"
=======
#include <eigen3/Eigen/Geometry>
#include <expected>
#include <yaml-cpp/yaml.h>

#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"
>>>>>>> c2da77f (feat(logic): implement target selection logic and decouple from visualization)

namespace rmcs::kernel {

class FireControl {
<<<<<<< HEAD
    using Clock = util::Clock;

    RMCS_PIMPL_DEFINITION(FireControl)

public:
    struct Result {
        double pitch;
        double yaw;
        double horizon_distance;
    };

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto set_bullet_speed(double speed) -> void;

    auto solve(const predictor::Snapshot& snapshot, Translation const& odom_to_muzzle_translation)
        -> std::optional<Result>;
=======
    RMCS_PIMPL_DEFINITION(FireControl)

public:
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto choose_armor(std::span<const Armor3D> armors, const Eigen::Vector<double, 11>& ekf_x)
        -> std::optional<Armor3D>;
>>>>>>> c2da77f (feat(logic): implement target selection logic and decouple from visualization)
};
}

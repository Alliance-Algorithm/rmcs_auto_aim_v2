#pragma once
#include "module/tracker/trackable.hpp"
#include "utility/pimpl.hpp"

#include <optional>

#include <yaml-cpp/yaml.h>

namespace rmcs::kernel {

class FireControllerV2 {
    RMCS_PIMPL_DEFINITION(FireControllerV2)

public:
    struct State {
        Timestamp timestamp = { };

        double yaw   = 0.;
        double pitch = 0.;

        double max_yaw_vel = 10.0;
        double max_yaw_acc = 200.0;
    };

    struct Aimed {
        double yaw   = 0;
        double pitch = 0;

        bool shoot   = false;
        bool pre_aim = false;

        Point3d center = { };
        Point3d attack = { };
    };

    explicit FireControllerV2(const YAML::Node&);

    auto update(State state) -> void;

    auto aim(const Trackable&) -> std::optional<Aimed>;
};

}

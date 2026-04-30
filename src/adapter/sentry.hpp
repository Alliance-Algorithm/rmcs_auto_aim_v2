#pragma once

#include <eigen3/Eigen/Geometry>
#include <rmcs_description/sentry_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs {

class Adapter {
public:
    explicit Adapter(rmcs_executor::Component& component) { component.register_input("/tf", tf_); }

    [[nodiscard]] auto ready() const -> bool { return tf_.ready(); }

    [[nodiscard]] auto camera_transform() const -> Eigen::Isometry3d {
        return fast_tf::lookup_transform<rmcs_description::OdomGimbalImu,
            rmcs_description::CameraLink>(*tf_);
    }

    [[nodiscard]] auto barrel_direction() const -> Eigen::Vector3d {
        return *fast_tf::cast<rmcs_description::OdomGimbalImu>(
            rmcs_description::PitchLink::DirectionVector { Eigen::Vector3d::UnitX() }, *tf_);
    }

private:
    rmcs_executor::Component::InputInterface<rmcs_description::SentryTf> tf_;
};

}

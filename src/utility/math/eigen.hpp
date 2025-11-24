#pragma once
#include "linear.hpp"
#include <eigen3/Eigen/Geometry>

namespace rmcs::util {

constexpr auto operator<<(Translation& translation, Eigen::Isometry3d& iso) noexcept
    -> Translation& {
    translation = iso.translation();
    return translation;
}
constexpr auto operator<<(Orientation& orientation, Eigen::Isometry3d& iso) noexcept
    -> Orientation& {
    orientation = Eigen::Quaterniond { iso.linear() };
    return orientation;
}

constexpr auto operator<<(Eigen::Isometry3d& iso, Translation& translation) noexcept
    -> Eigen::Isometry3d& {
    iso.translation()[0] = translation.x;
    iso.translation()[1] = translation.y;
    iso.translation()[2] = translation.z;
    return iso;
}
constexpr auto operator<<(Eigen::Isometry3d& iso, Orientation& orientation) noexcept
    -> Eigen::Isometry3d& {
    iso.rotate(orientation.make<Eigen::Quaterniond>().toRotationMatrix());
    return iso;
}

}

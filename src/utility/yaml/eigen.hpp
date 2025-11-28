#pragma once
#include <eigen3/Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace rmcs::util {

/// @note: x, y, z
template <typename T>
inline auto read_eigen_translation(const std::array<T, 3>& data) {
    return Eigen::Vector3<T> { data[0], data[1], data[2] };
}

/// @note: x, y, z
template <typename T>
inline auto read_eigen_translation(const YAML::Node& data) {
    const auto& array = data.as<std::array<T, 3>>();
    return read_eigen_translation<T>(array);
}

/// @note: w, x, y, z
template <typename T>
inline auto read_eigen_orientation(const std::array<T, 4>& data) {
    return Eigen::Quaternion<T> { data[0], data[1], data[2], data[3] };
}

/// @note: w, x, y, z
template <typename T>
inline auto read_eigen_orientation(const YAML::Node& data) {
    const auto& array = data.as<std::array<T, 4>>();
    return read_eigen_orientation<T>(array);
}

}

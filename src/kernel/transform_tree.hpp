#pragma once
#include "utility/tf/static_tf.hpp"
#include <eigen3/Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace rmcs::tf {

using namespace rmcs::util;

constexpr auto standard_transform_tree = Joint {
    Link<"world_link">(),
    Joint {
        Link<"odom_link", Eigen::Isometry3d>(),
        Joint {
            Link<"imu_link", Eigen::Quaterniond>(),
            Joint {
                Link<"gimbal_link", Eigen::Isometry3d>(),
                Joint {
                    Link<"camera_link", Eigen::Isometry3d>(),
                    Joint {
                        Link<"target_armor_link", Eigen::Isometry3d>(),
                    },
                    Joint {
                        Link<"target_buff_link", Eigen::Isometry3d>(),
                    },
                },
            },
        },
    },
};
using AutoAim = decltype(standard_transform_tree);

}

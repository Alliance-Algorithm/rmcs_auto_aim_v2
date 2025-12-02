#pragma once
#include "utility/tf/static_tf.hpp"
#include <eigen3/Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace rmcs::tf {

using namespace rmcs::util;

constexpr auto standard_transform_tree = Joint {
    Link<"world_link">(),

    Joint {
        Link<"odom_link">(),

        Joint {
            Link<"imu_link", Eigen::Quaterniond>(),

            Joint {
                Link<"pitch_link", Eigen::Quaterniond>(),

                Joint {
                    Link<"muzzle_link", Eigen::Isometry3d>(),
                },
                Joint {
                    Link<"camera_link", Eigen::Isometry3d>(),
                },
                Joint {
                    Link<"yaw_link", Eigen::Quaterniond>(),

                    Joint {
                        Link<"gimbal_center_link", Eigen::Isometry3d>(),

                        Joint {
                            Link<"base_link", Eigen::Isometry3d>(),

                            Joint {
                                Link<"left_front_wheel_link", Eigen::Vector3d>(),
                            },
                            Joint {
                                Link<"right_front_wheel_link", Eigen::Vector3d>(),
                            },
                            Joint {
                                Link<"left_back_wheel_link", Eigen::Vector3d>(),
                            },
                            Joint {
                                Link<"right_back_wheel_link", Eigen::Vector3d>(),
                            },
                        },
                    },
                },
            },
        },
    },
};
using AutoAim = decltype(standard_transform_tree);

}

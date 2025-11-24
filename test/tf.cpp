#include "utility/tf.hpp"
#include <eigen3/Eigen/Geometry>
#include <gtest/gtest.h>

using namespace rmcs;
using namespace rmcs::util;

constexpr static auto tree = TfJoint {
    "base_link",
    TfJoint {
        "gimbal_link",
        TfJoint { "scope_link" },
        TfJoint { "lidar_link" },
        TfJoint { "camera_link" },
    },
    TfJoint {
        "chassis_link",
        TfJoint { "wheel0_link" },
        TfJoint { "wheel1_link" },
        TfJoint { "wheel2_link" },
        TfJoint { "wheel3_link" },
    },
    TfJoint {
        "arm_link",
        TfJoint {
            "shoulder_link",
            TfJoint {
                "elbow_link",
                TfJoint {
                    "wrist_link",
                    TfJoint { "gripper_link" },
                },
            },
        },
    },
    TfJoint {
        "sensor_link",
        TfJoint { "imu_link" },
        TfJoint { "gps_link" },
        TfJoint {
            "radar_link",
            TfJoint { "antenna_link" },
        },
    },
};
using JointRoot = decltype(tree);

//
// 测试用例
//

TEST(tf, tree_structure) {
    static_assert(JointRoot::kTotalAmount == 20);
    static_assert(JointRoot::kChildAmount == 4);
    static_assert(tree.has_duplicate_nodes() == false);
}

TEST(tf, contains_and_find) {
    static_assert(tree.contains("wheel0_link"));
    static_assert(tree.contains("gripper_link"));
    static_assert(tree.contains("antenna_link"));
    static_assert(tree.find("imu_link", [](const auto&) { }));
}

TEST(tf, distance_single) {
    static_assert(tree.child_distance("chassis_link") == 1);
    static_assert(tree.child_distance("arm_link") == 1);
    static_assert(tree.child_distance("base_link", "wheel0_link") == 2);
    static_assert(tree.child_distance("base_link", "gripper_link") == 5);
    static_assert(tree.child_distance("sensor_link", "antenna_link") == 2);
}

TEST(tf, path_to_child) {
    constexpr auto dist    = tree.child_distance("base_link", "wheel0_link");
    constexpr auto tf_path = tree.find_child_path<dist + 1>("base_link", "wheel0_link");
    static_assert(tf_path->at(0) == "wheel0_link");
    static_assert(tf_path->at(1) == "chassis_link");
    static_assert(tf_path->at(2) == "base_link");

    constexpr auto dist2    = tree.child_distance("base_link", "gripper_link");
    constexpr auto tf_path2 = tree.find_child_path<dist2 + 1>("base_link", "gripper_link");
    static_assert(tf_path2->at(0) == "gripper_link");
    static_assert(tf_path2->at(1) == "wrist_link");
    static_assert(tf_path2->at(2) == "elbow_link");
    static_assert(tf_path2->at(3) == "shoulder_link");
    static_assert(tf_path2->at(4) == "arm_link");
    static_assert(tf_path2->at(5) == "base_link");
}

TEST(tf, nearest_common_ancestor) {
    constexpr auto ancestor = tree.find_nearest_common_ancestor("wheel0_link", "scope_link");
    static_assert(ancestor == "base_link");

    constexpr auto ancestor2 = tree.find_nearest_common_ancestor("gripper_link", "imu_link");
    static_assert(ancestor2 == "base_link");

    constexpr auto ancestor3 = tree.find_nearest_common_ancestor("antenna_link", "gps_link");
    static_assert(ancestor3 == "sensor_link");
}

TEST(tf, find_path_examples) {
    {
        constexpr auto path = tree.find_path("wheel0_link", "scope_link");
        static_assert(path.has_value());

        constexpr auto array = path.value();
        static_assert(array[0] == "wheel0_link");
        static_assert(array[2] == "base_link");
        static_assert(array[4] == "scope_link");
    }
    {
        constexpr auto path = tree.find_path("gripper_link", "imu_link");
        static_assert(path.has_value());

        constexpr auto array = path.value();
        static_assert(array[0] == "gripper_link");
        static_assert(array[1] == "wrist_link");
        static_assert(array[2] == "elbow_link");
        static_assert(array[3] == "shoulder_link");
        static_assert(array[4] == "arm_link");
        static_assert(array[5] == "base_link");
        static_assert(array[6] == "sensor_link");
        static_assert(array[7] == "imu_link");
    }
    {
        constexpr auto path = tree.find_path("antenna_link", "gps_link");
        static_assert(path.has_value());

        constexpr auto array = path.value();
        static_assert(array[0] == "antenna_link");
        static_assert(array[1] == "radar_link");
        static_assert(array[2] == "sensor_link");
        static_assert(array[3] == "gps_link");
    }
    {
        constexpr auto path = tree.find_path("base_link", "gripper_link");
        static_assert(path.has_value());

        constexpr auto array = path.value();
        static_assert(array[0] == "base_link");
        static_assert(array[1] == "arm_link");
        static_assert(array[5] == "gripper_link");
    }
}

TEST(tf, look_up_transform) {
    constexpr static auto tree = TfJoint {
        JointStatus { "base" },
        TfJoint {
            JointStatus { "link1" },
            TfJoint { JointStatus { "link2" } },
        },
    };

    JointTransforms jt { tree };

    jt["base"].translation = Eigen::Vector3d { 0, 0, 0 };
    jt["base"].orientation = Eigen::Quaterniond::Identity();

    jt["link1"].translation = Eigen::Vector3d { 1, 0, 0 };
    jt["link1"].orientation = Eigen::Quaterniond::Identity();

    jt["link2"].translation = Eigen::Vector3d { 0, 2, 0 };
    jt["link2"].orientation = Eigen::Quaterniond::Identity();

    // 测试：base -> link2 (向下，parent -> child)
    // transforms[child] 存储的是从 child 到 parent 的变换
    // base -> link1: 使用 link1 的逆变换 (-1, 0, 0)
    // link1 -> link2: 使用 link2 的逆变换 (0, -2, 0)
    // 组合: (-1, 0, 0) * (0, -2, 0) = (-1, -2, 0)
    {
        auto transform = jt.look_up("base", "link2");
        auto expected  = Eigen::Vector3d { -1, -2, 0 };
        EXPECT_EQ(transform.translation.make<Eigen::Vector3d>(), expected);
    }

    // 测试：link2 -> base (向上，child -> parent)
    // link2 -> link1: 使用 link2 的变换 (0, 2, 0)
    // link1 -> base: 使用 link1 的变换 (1, 0, 0)
    // 组合: (0, 2, 0) * (1, 0, 0) = (1, 2, 0)
    {
        auto transform = jt.look_up("link2", "base");
        auto expected  = Eigen::Vector3d { 1, 2, 0 };
        EXPECT_EQ(transform.translation.make<Eigen::Vector3d>(), expected);
    }

    // 测试：link1 -> link2 (向下，parent -> child)
    // 使用 link2 的逆变换 (0, -2, 0)
    {
        auto transform = jt.look_up("link1", "link2");
        auto expected  = Eigen::Vector3d { 0, -2, 0 };
        EXPECT_EQ(transform.translation.make<Eigen::Vector3d>(), expected);
    }

    // 测试：link2 -> link1 (向上，child -> parent)
    // 使用 link2 的变换 (0, 2, 0)
    {
        auto transform = jt.look_up("link2", "link1");
        auto expected  = Eigen::Vector3d { 0, 2, 0 };
        EXPECT_EQ(transform.translation.make<Eigen::Vector3d>(), expected);
    }

    // 测试：相同节点应该返回单位变换
    {
        auto transform = jt.look_up("base", "base");
        EXPECT_EQ(transform.translation.make<Eigen::Vector3d>(), Eigen::Vector3d::Zero());
        EXPECT_TRUE(transform.orientation.make<Eigen::Quaterniond>().isApprox(
            Eigen::Quaterniond::Identity()));
    }
}

TEST(tf, look_up_transform_with_rotation) {
    constexpr static auto tree = TfJoint {
        JointStatus { "base" },
        TfJoint {
            JointStatus { "link1" },
            TfJoint { JointStatus { "link2" } },
        },
    };

    JointTransforms jt { tree };

    jt["link1"].translation = Eigen::Vector3d { 0, 0, 0 };
    jt["link1"].orientation =
        Eigen::Quaterniond { Eigen::AngleAxisd(std::numbers::pi / 3, Eigen::Vector3d::UnitZ()) };

    jt["link2"].translation = Eigen::Vector3d { 0, 0, 0 };
    jt["link2"].orientation =
        Eigen::Quaterniond { Eigen::AngleAxisd(std::numbers::pi / 3, Eigen::Vector3d::UnitZ()) };

    {
        auto transform = jt.look_up("link2", "base");
        auto rotation  = transform.orientation.make<Eigen::Quaterniond>();
        auto expected  = Eigen::Quaterniond {
            Eigen::AngleAxisd(2 * std::numbers::pi / 3, Eigen::Vector3d::UnitZ()),
        };
        EXPECT_NEAR(std::abs(rotation.angularDistance(expected)), 0, 1e-4);
    }
    {
        auto transform = jt.look_up("base", "link2");
        auto rotation  = transform.orientation.make<Eigen::Quaterniond>();
        auto expected  = Eigen::Quaterniond {
            Eigen::AngleAxisd(-2 * std::numbers::pi / 3, Eigen::Vector3d::UnitZ()),
        };
        EXPECT_NEAR(std::abs(rotation.angularDistance(expected)), 0, 1e-4);
    }
}

TEST(tf, look_up_transform_complex_tree) {
    constexpr static auto complex_tree = TfJoint {
        JointStatus { "base_link" },
        TfJoint {
            JointStatus { "arm_link" },
            TfJoint {
                JointStatus { "shoulder_link" },
                TfJoint {
                    JointStatus { "elbow_link" },
                    TfJoint { JointStatus { "wrist_link" } },
                },
            },
        },
        TfJoint {
            JointStatus { "sensor_link" },
            TfJoint { JointStatus { "imu_link" } },
            TfJoint { JointStatus { "gps_link" } },
        },
    };

    JointTransforms jt { complex_tree };

    jt["base_link"].translation = Eigen::Vector3d { 0, 0, 0 };
    jt["base_link"].orientation = Eigen::Quaterniond::Identity();

    jt["arm_link"].translation = Eigen::Vector3d { 1, 0, 0 };
    jt["arm_link"].orientation = Eigen::Quaterniond::Identity();

    jt["shoulder_link"].translation = Eigen::Vector3d { 0, 1, 0 };
    jt["shoulder_link"].orientation = Eigen::Quaterniond::Identity();

    jt["elbow_link"].translation = Eigen::Vector3d { 0, 0, 1 };
    jt["elbow_link"].orientation = Eigen::Quaterniond::Identity();

    jt["wrist_link"].translation = Eigen::Vector3d { 0, 0, 1 };
    jt["wrist_link"].orientation = Eigen::Quaterniond::Identity();

    jt["sensor_link"].translation = Eigen::Vector3d { 2, 0, 0 };
    jt["sensor_link"].orientation = Eigen::Quaterniond::Identity();

    jt["imu_link"].translation = Eigen::Vector3d { 0, 0.5, 0 };
    jt["imu_link"].orientation = Eigen::Quaterniond::Identity();

    jt["gps_link"].translation = Eigen::Vector3d { 0, -0.5, 0 };
    jt["gps_link"].orientation = Eigen::Quaterniond::Identity();

    {
        auto transform = jt.look_up("base_link", "wrist_link");
        auto expected  = Eigen::Vector3d { -1, -1, -2 }; // 所有逆变换的组合
        EXPECT_EQ(transform.translation.make<Eigen::Vector3d>(), expected);
    }
    {
        auto transform = jt.look_up("wrist_link", "base_link");
        auto expected  = Eigen::Vector3d { 1, 1, 2 }; // 所有变换的组合
        EXPECT_EQ(transform.translation.make<Eigen::Vector3d>(), expected);
    }
    {
        auto transform = jt.look_up("wrist_link", "imu_link");
        auto expected  = Eigen::Vector3d { -1, 0.5, 2 };
        auto actual    = transform.translation.make<Eigen::Vector3d>();
        EXPECT_NEAR(actual.x(), expected.x(), 1e-6);
        EXPECT_NEAR(actual.y(), expected.y(), 1e-6);
        EXPECT_NEAR(actual.z(), expected.z(), 1e-6);
    }
    {
        auto transform = jt.look_up("imu_link", "gps_link");
        auto expected  = Eigen::Vector3d { 0, 1, 0 };
        EXPECT_EQ(transform.translation.make<Eigen::Vector3d>(), expected);
    }
    {
        auto transform = jt.look_up("elbow_link", "shoulder_link");
        auto expected  = Eigen::Vector3d { 0, 0, 1 };
        EXPECT_EQ(transform.translation.make<Eigen::Vector3d>(), expected);
    }
}

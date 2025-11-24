#include "utility/static_tf.hpp"
#include <eigen3/Eigen/Geometry>
#include <gtest/gtest.h>
#include <print>

using namespace rmcs::util::tf::details;
using namespace rmcs::util;

TEST(static_tf, construct) {

    constexpr auto static_tf = Joint {
        JointState<"0">(),

        Joint {
            JointState<"0.0", Eigen::Quaterniond>(),

            Joint {
                JointState<"0.0.0", Eigen::Quaterniond>(),

                Joint { JointState<"0.0.0.0", Eigen::Quaterniond>() },
                Joint { JointState<"0.0.0.1", Eigen::Quaterniond>() },
                Joint { JointState<"0.0.0.2", Eigen::Quaterniond>() },
            },
            Joint { JointState<"0.0.1", Eigen::Quaterniond>() },
        },
        Joint {
            JointState<"0.1", Eigen::Quaterniond>(),

            Joint { JointState<"0.1.0", Eigen::Quaterniond>() },
            Joint { JointState<"0.1.1", Eigen::Quaterniond>() },
        },
    };
    using StaticTf = decltype(static_tf);

    StaticTf::foreach_df([]<typename T>() {
        // Use static function
        std::println("name: {}", T::name);
    });

    static_assert(StaticTf::name == "0");
    static_assert(StaticTf::child_amount > 0);
    static_assert(StaticTf::total_amount > 0);

    static_assert(StaticTf::contains<"0.1.1">());
    static_assert(StaticTf::child_distance<"0.1.1">() == 2);
    static_assert(StaticTf::child_distance<"0", "0.1.1">() == 2);

    {
        constexpr auto traversal_down { true };
        constexpr auto result = StaticTf::child_path<"0", "0.0.0.2">(traversal_down);
        static_assert(result.size() == 3);
        static_assert(result.at(0) == "0.0");
        static_assert(result.at(1) == "0.0.0");
        static_assert(result.at(2) == "0.0.0.2");
    }
    {
        constexpr auto result = StaticTf::find_lca<"0.0.0.2", "0.0.1">();
        static_assert(std::get<0>(result) == "0.0");
        static_assert(std::get<1>(result) == 3);
    }
    {
        constexpr auto result = StaticTf::path<"0.0.0.2", "0.0.1">();
        static_assert(result.size() == 3);
        static_assert(result.at(0) == "0.0.0.2");
        static_assert(result.at(1) == "0.0.0");
        static_assert(result.at(2) == "0.0.1");
    }
    {
        auto state = StaticTf::look_up<"0.0.0", Eigen::Quaterniond>();
        //
    }
}

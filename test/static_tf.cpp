#include "utility/tf/static_tf.hpp"
#include "utility/logging/eigen.hpp" // IWYU pragma: keep
#include "utility/yaml/tf.hpp"
#include <eigen3/Eigen/Geometry>
#include <gtest/gtest.h>
#include <print>
#include <yaml-cpp/yaml.h>

using namespace rmcs::util;

constexpr auto static_tf = Joint {
    Link<"0">(),
    Joint {
        Link<"0.0", Eigen::Isometry3d>(),
        Joint {
            Link<"0.0.0", Eigen::Isometry3d>(),
            Joint { Link<"0.0.0.0", Eigen::Isometry3d>() },
            Joint { Link<"0.0.0.1", Eigen::Isometry3d>() },
            Joint { Link<"0.0.0.2", Eigen::Isometry3d>() },
        },
        Joint { Link<"0.0.1", Eigen::Isometry3d>() },
    },
    Joint {
        Link<"0.1", Eigen::Isometry3d>(),
        Joint { Link<"0.1.0", Eigen::Isometry3d>() },
        Joint { Link<"0.1.1", Eigen::Isometry3d>() },
    },
};
using SentryTf = decltype(static_tf);

TEST(static_tf, construct) {
    SentryTf::foreach_df_with_parent(
        []<class T>(auto parent) { std::println("{} -> {}", parent, T::name); });

    using Result = SentryTf::Find<"0.0.0">::Result;

    constexpr auto result = SentryTf::find<"0.0.0">();

    static_assert(SentryTf::name == "0");
    static_assert(SentryTf::child_amount > 0);
    static_assert(SentryTf::total_amount > 0);

    static_assert(SentryTf::contains<"0.1.1">());
    static_assert(SentryTf::child_distance<"0.1.1">() == 2);
    static_assert(SentryTf::child_distance<"0", "0.1.1">() == 2);

    {
        constexpr auto result = SentryTf::find<"0.0.0">();
        static_assert(!result.contains<"0.0">());
    }
    {
        constexpr auto traversal_down { true };
        constexpr auto result = SentryTf::child_path<"0", "0.0.0.2">(traversal_down);
        static_assert(result.size() == 3);
        static_assert(result.at(0) == "0.0");
        static_assert(result.at(1) == "0.0.0");
        static_assert(result.at(2) == "0.0.0.2");
    }
    {
        constexpr auto result = SentryTf::find_lca<"0.0.0.2", "0.0.1">();
        static_assert(std::get<0>(result) == "0.0");
    }
    {
        constexpr auto result = SentryTf::path<"0.0.0.2", "0.0.1">();
        static_assert(result.size() == 3);
        static_assert(result.at(0) == "0.0.0.2");
        static_assert(result.at(1) == "0.0.0");
        static_assert(result.at(2) == "0.0.1");
    }
}

auto expect_begin = Eigen::Isometry3d::Identity();
auto result_begin = Eigen::Isometry3d::Identity();
TEST(static_tf, look_up_begin) {
    {
        auto transfrom = Eigen::Translation3d { 1, 15, -10 };
        SentryTf::set_state<"0.1">(transfrom);
        expect_begin = transfrom * expect_begin;
    }
    {
        auto transfrom = Eigen::AngleAxisd { std::numbers::pi, Eigen::Vector3d::UnitZ() };
        SentryTf::set_state<"0.1.1">(transfrom);
        expect_begin = transfrom * expect_begin;
    }
    SentryTf::impl_look_up<"0.1.1", "0", Eigen::Isometry3d>(
        [&](auto name, const Eigen::Isometry3d& se3, auto is_begin) {
            std::println("[{} | {:10}]", is_begin ? "begin" : "final", name);
            std::println("  t: {:.2}", Eigen::Vector3d { se3.translation() });
            std::println("  q: {:.2}", Eigen::Quaterniond { se3.linear() });

            result_begin = se3.inverse() * result_begin;
        });
    result_begin = result_begin.inverse();

    std::println("result: {:.2}", result_begin);
    std::println("expect: {:.2}", expect_begin);
    EXPECT_TRUE(result_begin.isApprox(expect_begin));
}

auto expect_final = Eigen::Isometry3d::Identity();
auto result_final = Eigen::Isometry3d::Identity();
TEST(static_tf, loop_up_final) {
    {
        auto transfrom = Eigen::Translation3d { 1, 0, 0 };
        SentryTf::set_state<"0.0">(transfrom);
        expect_final = transfrom * expect_final;
    }
    {
        auto transfrom = Eigen::AngleAxisd { 0.5 * std::numbers::pi, Eigen::Vector3d::UnitX() };
        SentryTf::set_state<"0.0.0">(transfrom);
        expect_final = transfrom * expect_final;
    }
    {
        auto transfrom = Eigen::Translation3d { 0, 0, 2.5 };
        SentryTf::set_state<"0.0.0.0">(transfrom);
        expect_final = transfrom * expect_final;
    }
    SentryTf::impl_look_up<"0", "0.0.0.0", Eigen::Isometry3d>(
        [&](auto name, const Eigen::Isometry3d& se3, auto is_begin) {
            std::println("[{} | {:10}]", is_begin ? "begin" : "final", name);
            std::println("  t: {:.2}", Eigen::Vector3d { se3.translation() });
            std::println("  q: {:.2}", Eigen::Quaterniond { se3.linear() });

            result_final = se3.inverse() * result_final;
        });
    result_final = result_final.inverse();

    std::println("result: {:.2}", result_final);
    std::println("expect: {:.2}", expect_final);
    EXPECT_TRUE(result_final.isApprox(expect_final));
}

TEST(static_tf, look_up) {
    auto expect = expect_begin.inverse() * expect_final;
    auto result = SentryTf::look_up<"0.1.1", "0.0.0.0", Eigen::Isometry3d>();

    std::println("result: {:.2}", result);
    std::println("expect: {:.2}", expect);
    EXPECT_TRUE(result.isApprox(expect));

    SentryTf::look_up<"0", "0.0", Eigen::Isometry3d>();
}

TEST(static_tf, serialize_from_yaml) {

    SentryTf::set_state<"0.0">(Eigen::Isometry3d::Identity());
    SentryTf::set_state<"0.0.0">(Eigen::Isometry3d::Identity());
    SentryTf::set_state<"0.0.0.0">(Eigen::Isometry3d::Identity());
    SentryTf::set_state<"0.1">(Eigen::Isometry3d::Identity());
    SentryTf::set_state<"0.1.1">(Eigen::Isometry3d::Identity());

    constexpr auto yaml_str = R"(
        - parent: "0"
          child: "0.0"
          t: [1.0, 0.0, 0.0]
          q: [1.0, 0.0, 0.0, 0.0]

        - parent: "0.0"
          child: "0.0.0"
          t: [0.0, 0.0, 2.5]
          q: [0.707, 0.707, 0.0, 0.0]

        - parent: "0.0.0"
          child: "0.0.0.0"
          t: [0.0, 0.0, 1.0]
          q: [1.0, 0.0, 0.0, 0.0]

        - parent: "0"
          child: "0.1"
          t: [1.0, 15.0, -10.0]
          q: [1.0, 0.0, 0.0, 0.0]

        - parent: "0.1"
          child: "0.1.1"
          q: [0.0, 0.0, 0.0, 1.0]
    )";

    auto yaml = YAML::Load(yaml_str);

    // 执行序列化
    auto result = serialize_from<SentryTf>(yaml);
    EXPECT_TRUE(result.has_value());

    // 验证序列化结果
    auto tf_0_0 = SentryTf::get_state<"0.0", Eigen::Isometry3d>();
    EXPECT_NEAR(tf_0_0.translation().x(), 1.0, 1e-6);
    EXPECT_NEAR(tf_0_0.translation().y(), 0.0, 1e-6);
    EXPECT_NEAR(tf_0_0.translation().z(), 0.0, 1e-6);

    auto tf_0_0_0 = SentryTf::get_state<"0.0.0", Eigen::Isometry3d>();
    EXPECT_NEAR(tf_0_0_0.translation().z(), 2.5, 1e-6);
    auto q_0_0_0 = Eigen::Quaterniond { tf_0_0_0.linear() };
    EXPECT_NEAR(q_0_0_0.w(), 0.707, 1e-3);
    EXPECT_NEAR(q_0_0_0.x(), 0.707, 1e-3);

    auto tf_0_1 = SentryTf::get_state<"0.1", Eigen::Isometry3d>();
    EXPECT_NEAR(tf_0_1.translation().x(), 1.0, 1e-6);
    EXPECT_NEAR(tf_0_1.translation().y(), 15.0, 1e-6);
    EXPECT_NEAR(tf_0_1.translation().z(), -10.0, 1e-6);

    auto tf_0_1_1 = SentryTf::get_state<"0.1.1", Eigen::Isometry3d>();
    auto q_0_1_1  = Eigen::Quaterniond { tf_0_1_1.linear() };
    EXPECT_NEAR(q_0_1_1.w(), 0.0, 1e-6);
    EXPECT_NEAR(q_0_1_1.z(), 1.0, 1e-6);
}

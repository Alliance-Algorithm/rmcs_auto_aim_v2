#include "utility/serializable.hpp"
#include <filesystem>
#include <gtest/gtest.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/parameter_map.hpp>

constexpr auto kFilePath = __FILE__;

TEST(serializable, rclcpp) {
    using namespace rmcs::util;

    const auto current_file = std::filesystem::path(kFilePath);
    const auto current_path = current_file.parent_path();
    const auto current_yaml = current_path / "serializable.yaml";

    const auto argv = std::array<const char*, 4> {
        "ignored-executable-name",
        "--ros-args",
        "--params-file",
        current_yaml.c_str(),
    };
    const auto argc = int { sizeof(argv) / sizeof(argv[0]) };

    rclcpp::init(argc, argv.data());
    auto options = rclcpp::NodeOptions {}.automatically_declare_parameters_from_overrides(true);
    auto node    = std::make_shared<rclcpp::Node>("serializable", options);

    struct T {
        int mem1;
        std::string mem2;
        double mem3;
        std::vector<double> mem4;
    } t;
    constexpr auto TFactory = Serializable {
        MemberMeta { &T::mem1, "mem1" },
        MemberMeta { &T::mem2, "mem2" },
        MemberMeta { &T::mem3, "mem3" },
        MemberMeta { &T::mem4, "mem4" },
    };

    auto ret = TFactory.serialize("test.", *node, t);
    if (!ret.has_value()) {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", ret.error().c_str());
        GTEST_FAIL();
    }

    const auto printable = TFactory.make_printable_from(t);
    RCLCPP_INFO(node->get_logger(), "Print Data:\n%s", printable.c_str());

    rclcpp::shutdown();
}

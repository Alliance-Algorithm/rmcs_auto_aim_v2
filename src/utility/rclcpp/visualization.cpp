#include "visualization.hpp"
#include "utility/math/solve_armors.hpp"
#include "utility/panic.hpp"
#include "utility/robot/id.hpp"
#include <rclcpp/node.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace rmcs::util {

using Marker = visualization_msgs::msg::Marker;

namespace visual::details {
    struct Context::Details {
        Marker marker_status;
        std::shared_ptr<rclcpp::Publisher<Marker>> rclcpp_pub;
    };
    auto Context::update() noexcept -> void {
        std::ignore = std::as_const(*this);

        details->marker_status.header.stamp = rclcpp::Clock { RCL_SYSTEM_TIME }.now();
        details->rclcpp_pub->publish(details->marker_status);
    }

    auto Context::share_rclcpp_context(Context& o) const noexcept -> void {
        o.details->rclcpp_pub = details->rclcpp_pub;
    }

    Context::Context() noexcept
        : details { std::make_unique<Details>() } { }

    Context::~Context() noexcept = default;

}
namespace visual {
    static auto init_armor_context(
        details::Context& context, DeviceId device_id, CampColor camp_color) {

        auto& marker = context.details->marker_status;

        marker.header.frame_id = context.tf;
        marker.header.stamp    = rclcpp::Clock { RCL_SYSTEM_TIME }.now();

        marker.ns     = context.id;
        marker.id     = 0;
        marker.type   = Marker::CUBE;
        marker.action = Marker::ADD;

        // ref: "https://www.robomaster.com/zh-CN/products/components/detail/149"
        /*  */ if (DeviceIds::kSmallArmorDevices().contains(device_id)) {
            marker.scale.x = 0.003, marker.scale.y = 0.140, marker.scale.z = 0.125;
        } else if (DeviceIds::kLargeArmorDevices().contains(device_id)) {
            marker.scale.x = 0.003, marker.scale.y = 0.235, marker.scale.z = 0.127;
        } else {
            util::panic("Wrong device id for a visualized armor");
        };

        /*  */ if (camp_color == CampColor::RED) {
            marker.color.r = 1., marker.color.g = 0., marker.color.b = 0., marker.color.a = 1.;
        } else if (camp_color == CampColor::BLUE) {
            marker.color.r = 0., marker.color.g = 0., marker.color.b = 1., marker.color.a = 1.;
        } else {
            util::panic("Please specify a valid armor color");
        }
    }

    auto Armor::update() noexcept -> void { context.update(); }

    auto Armor::set_translation(const Translation& translation) noexcept -> void {
        std::ignore = std::as_const(*this);
        translation.copy_to(context.details->marker_status.pose.position);
    }
    auto Armor::set_orientation(const Orientation& orientation) noexcept -> void {
        std::ignore = std::as_const(*this);
        orientation.copy_to(context.details->marker_status.pose.orientation);
    }
    auto Armor::init_context() noexcept -> void {
        init_armor_context(context, device_id, camp_color);
    }

    auto AssembledArmors::init(
        DeviceId device_id, CampColor camp_color, double w, double h) noexcept -> void {
        this->w = w, this->h = h;
        for (auto& armor : armors) {
            context.share_rclcpp_context(armor.context);
            armor.init(device_id, camp_color);
        }
    }
    auto AssembledArmors::update() noexcept -> void {
        auto solution = ArmorsForwardSolution {};

        solution.input.robot_height = h;
        solution.input.robot_width  = w;

        solution.input.t = translation;
        solution.input.q = orientation;

        solution.solve();

        const auto& armors_status = solution.result.armors_status;
        for (auto&& [armor, status] : std::views::zip(armors, armors_status)) {
            const auto& [t, q] = status;
            armor.set_translation(t);
            armor.set_orientation(q);
            armor.update();
        }
    }
}

struct Visualization::Impl {

    std::shared_ptr<rclcpp::Node> rclcpp;

    explicit Impl(const std::string& id) noexcept
        : rclcpp { std::make_shared<rclcpp::Node>(id) } { }

    auto bind_context(visual::details::Context& context) const noexcept -> void {
        auto prefix = std::string { "/rmcs/auto_aim/" };

        context.details->rclcpp_pub = rclcpp->create_publisher<Marker>(prefix + context.id, 10);
    }
};

auto Visualization::bind_context(visual::details::Context& context) noexcept -> void {
    pimpl->bind_context(context);
}

Visualization::Visualization(const std::string& id) noexcept
    : pimpl { std::make_unique<Impl>(id) } { }

Visualization::Visualization() noexcept { util::panic("Should not use this constructer!"); }

Visualization::~Visualization() noexcept = default;

}

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs {

static auto options = rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true);

class AutoAimComponent final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit AutoAimComponent() noexcept
        : Node{get_component_name(), options} {

        RCLCPP_INFO(get_logger(), "AutoAim Component is initializing now");
    }

    void update() override {}
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)
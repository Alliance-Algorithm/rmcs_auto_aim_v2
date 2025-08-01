#include "utility/rclcpp.hpp"
#include <rmcs_executor/component.hpp>

namespace rmcs {

class AutoAimComponent final
    : public rmcs_executor::Component
    , public utility::Node {
public:
    explicit AutoAimComponent() noexcept
        : Node{get_component_name(), utility::options} {

        rclcpp_info("{}", get_parameter_or<std::string>("msg", ""));
    }

    void update() override {}

private:
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)

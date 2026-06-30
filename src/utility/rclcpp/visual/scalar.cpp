#include "scalar.hpp"
#include "utility/rclcpp/node.details.hpp"

#include <std_msgs/msg/float64.hpp>

using namespace rmcs::util::visual;

using Float64 = std_msgs::msg::Float64;

struct Scalar::Impl {
    static inline rclcpp::Clock rclcpp_clock { RCL_STEADY_TIME };

    RclcppNode& node;
    std::string name;
    std::shared_ptr<rclcpp::Publisher<Float64>> publisher;

    explicit Impl(RclcppNode& node, std::string name) noexcept
        : node { node }
        , name { std::move(name) } { }

    auto publish(double value) -> void {
        if (!publisher) {
            auto topic_name = node.get_pub_topic_prefix() + name;
            publisher       = node.details->make_pub<Float64>(topic_name, qos::debug);
        }
        auto msg = Float64 { };
        msg.data = value;
        publisher->publish(msg);
    }
};

Scalar::Scalar(RclcppNode& node, std::string name)
    : pimpl { std::make_unique<Impl>(node, std::move(name)) } { }

Scalar::~Scalar() noexcept = default;

Scalar::Scalar(Scalar&&) noexcept                    = default;
auto Scalar::operator=(Scalar&&) noexcept -> Scalar& = default;

auto Scalar::publish(double value) -> void { pimpl->publish(value); }

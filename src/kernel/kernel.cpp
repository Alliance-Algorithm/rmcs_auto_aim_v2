#include "kernel.impl.hpp"

#include <hikcamera/image_capturer.hpp>

using namespace rmcs;

namespace v1 {
using Capturer = hikcamera::ImageCapturer;
using Identifier = void;
using Tracker = void;
using Kernel = internal::Kernel<Capturer, Identifier, Tracker>;
} // namespace v1

namespace v2 {
using Capturer = hikcamera::ImageCapturer;
using Identifier = void;
using Tracker = void;
using Kernel = internal::Kernel<Capturer, Identifier, Tracker>;
} // namespace v2

struct AutoAimKernel::Impl {
public:
    explicit Impl(rclcpp::Node& node) noexcept
        : node_{node} {
        //
        auto capturer = hikcamera::ImageCapturer{};

        rclcpp_info("AutoAim Kernel is initializing now");
    }

private:
    std::unique_ptr<v1::Kernel> kernel_;

    rclcpp::Node& node_;

    auto rclcpp_info(const std::string& msg) const noexcept -> void {
        RCLCPP_INFO(node_.get_logger(), "%s", msg.c_str());
    }
};
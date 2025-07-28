#include "kernel.hpp"

#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>

#include <hikcamera/image_capturer.hpp>

#include <string>

namespace rmcs::internal {

template <class T>
concept concept_capturer = requires(T& capturer) {
    { capturer.read(std::chrono::seconds(5)) } -> std::same_as<cv::Mat>;
};

template <concept_capturer Capturer>
class Kernel {
public:
    explicit Kernel(std::unique_ptr<Capturer> capturer) noexcept
        : capturer_{std::move(capturer)} {};

private:
    std::unique_ptr<Capturer> capturer_;
};

} // namespace rmcs::internal

struct rmcs::AutoAimKernel::Impl {
public:
    explicit Impl(rclcpp::Node& node) noexcept
        : node_{node} {
        //
        auto capturer = hikcamera::ImageCapturer{};

        rclcpp_info("AutoAim Kernel is initializing now");
    }

private:
    using Kernel = internal::Kernel<hikcamera::ImageCapturer>;
    std::unique_ptr<Kernel> kernel_;

    rclcpp::Node& node_;

    auto rclcpp_info(const std::string& msg) const noexcept -> void {
        RCLCPP_INFO(node_.get_logger(), "%s", msg.c_str());
    }
};
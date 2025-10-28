#include "kernel.hpp"
#include "modules/capturer/hikcamera.hpp"
#include "utility/coroutine/channel.hpp"
#include "utility/image.hpp"

using namespace rmcs;

struct AutoAimKernel::Impl {
public:
    explicit Impl(util::Node& node) noexcept
        : node { node } {
        node.info("AutoAim Kernal is initializing...");

        capturer = std::make_unique<cap::Hikcamera>();
        if (auto ret = capturer->initialize(); !ret) {
            node.error("Failed to initialize capturer");
            node.error("- Error: {}", ret.error());
        }
    }

    auto capturer_runtime() noexcept -> void {
        node.info("Capturer runtime starts");
        while (rclcpp::ok()) { }
    }
    auto inference_runtime() noexcept -> void {
        node.info("Inference runtime starts");
        while (rclcpp::ok()) { }
    }
    auto calculate_runtime() noexcept -> void {
        node.info("Calculate runtime starts");
        while (rclcpp::ok()) { }
    }
    auto transmit_runtime() noexcept -> void {
        node.info("Transmit runtime starts");
        while (rclcpp::ok()) { }
    }

private:
    util::Node& node;

    // Modules
    std::unique_ptr<cap::Hikcamera> capturer;

    // Channels
    co::Channel<Image> images_channel;
};

AutoAimKernel::AutoAimKernel() noexcept
    : Node { "AutoAim", util::options }
    , pimpl { std::make_unique<Impl>(*this) } { }

AutoAimKernel::~AutoAimKernel() noexcept = default;

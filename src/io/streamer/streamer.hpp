#pragma once

#include <memory>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs {

// Streams CameraFrame events without running the auto-aim pipeline.
class AutoAimStreamerComponent final : public rmcs_executor::Component, public rclcpp::Node {
public:
    AutoAimStreamerComponent();
    ~AutoAimStreamerComponent() override;

    void update() override;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rmcs

#pragma once

#include "utility/math/linear.hpp"
#include "utility/rclcpp/node.hpp"

#include <memory>
#include <string>

namespace rmcs::util::visual {

struct LightBar {
public:
    struct Config {
        RclcppNode& rclcpp;

        int id { 0 };
        std::string name { "lightbar" };
        std::string tf { "camera_link" };

        float r { 1.0F };
        float g { 0.0F };
        float b { 0.0F };
        float a { 1.0F };

        double width { 0.01 };
    };

    explicit LightBar(const Config&) noexcept;

    ~LightBar() noexcept;

    LightBar(const LightBar&)            = delete;
    LightBar& operator=(const LightBar&) = delete;

    auto set(const Point3d& top, const Point3d& bottom) noexcept -> void;
    auto update() noexcept -> void;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}

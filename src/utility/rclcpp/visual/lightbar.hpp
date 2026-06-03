#pragma once

#include "utility/math/linear.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/robot/armor.hpp"

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

        float r { 0.0F };
        float g { 0.0F };
        float b { 0.0F };
        float a { 1.0F };

        double width { 0.01 };
    };

    explicit LightBar(const Config&) noexcept;

    ~LightBar() noexcept;

    LightBar(const LightBar&)            = delete;
    LightBar& operator=(const LightBar&) = delete;

    auto set_point(const Point3d& top, const Point3d& bottom) noexcept -> void;

    auto set_color(auto r, auto g, auto b, auto a = 1.0) noexcept -> void {
        set_color(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b),
            static_cast<float>(a));
    }
    auto set_color(ArmorVisualColor color, float a = 1.0) noexcept -> void {
        const auto [r, g, b] = color;
        set_color(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), a);
    }
    auto set_color(float r, float g, float b, float a = 1.0) noexcept -> void;

    auto update() noexcept -> void;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}

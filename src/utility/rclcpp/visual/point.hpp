#pragma once

#include "utility/rclcpp/node.hpp"

#include <memory>
#include <span>
#include <string>

namespace rmcs::util::visual {

struct Points {
    struct Point {
        double x;
        double y;
        double z;

        float r { 1.0F };
        float g { 1.0F };
        float b { 0.0F };
        float a { 1.0F };
    };

    struct Config {
        RclcppNode& rclcpp;

        int id { 0 };
        std::string name { "points" };
        std::string tf { "odom_imu_link" };

        double size { 0.05 }; // 点直径（m），组级
    };

    explicit Points(const Config&) noexcept;

    ~Points() noexcept;

    Points(const Points&)            = delete;
    Points& operator=(const Points&) = delete;
    Points(Points&&) noexcept;
    Points& operator=(Points&&) noexcept;

    auto update(std::span<const Point> points) noexcept -> void;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}

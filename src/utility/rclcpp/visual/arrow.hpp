#pragma once

#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/visual/movable.hpp"

namespace rmcs::util::visual {

struct Arrow : public Movable {
    friend Movable;

public:
    struct Config {
        RclcppNode& rclcpp;

        int id { 0 };
        std::string name { "arrow" };
        std::string tf { "odom_link" };

        float r { 0. };
        float g { 1. };
        float b { 0. };
        float a { 1. };

        double length { 0.2 };
        double width { 0.01 };
        double height { 0.01 };
    };

    explicit Arrow(const Config&) noexcept;

    ~Arrow() noexcept;

    Arrow(const Arrow&)            = delete;
    Arrow& operator=(const Arrow&) = delete;

    auto update() noexcept -> void;

private:
    auto impl_move(const Translation&, const Orientation&) noexcept -> void;

    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}

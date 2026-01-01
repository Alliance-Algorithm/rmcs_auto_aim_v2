#pragma once
#include "utility/math/linear.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/visual/movable.hpp"

namespace rmcs::util::visual {

struct Transform : Movable {
    struct Config {
        RclcppNode& rclcpp;

        std::string topic;
        std::string parent_frame;
        std::string child_frame;
    };

    explicit Transform(const Config&) noexcept;
    ~Transform() noexcept;

    Transform(const Transform&)            = delete;
    Transform& operator=(const Transform&) = delete;

    auto update() noexcept -> void;

    auto impl_move(const Translation&, const Orientation&) noexcept -> void;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}

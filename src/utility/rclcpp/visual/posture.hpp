#pragma once
#include "utility/math/linear.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/visual/movable.hpp"

namespace rmcs::util::visual {

struct Posture : Movable {
    friend Movable;

public:
    struct Config {
        RclcppNode& rclcpp;

        std::string id;
        std::string tf;
    };
    explicit Posture(const Config&) noexcept;
    ~Posture() noexcept;

    Posture(const Posture&)            = delete;
    Posture& operator=(const Posture&) = delete;

    auto update() noexcept -> void;

private:
    auto impl_move(const Translation&, const Orientation&) noexcept -> void;

    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}

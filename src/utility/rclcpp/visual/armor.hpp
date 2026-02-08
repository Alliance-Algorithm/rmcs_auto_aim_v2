#pragma once
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/visual/movable.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::util::visual {

struct Armor : public Movable {
    friend Movable;

public:
    struct Config {
        RclcppNode& rclcpp;

        DeviceId device;
        CampColor camp;

        int id;
        std::string name;
        std::string tf;
    };

    explicit Armor(const Config&) noexcept;

    ~Armor() noexcept;

    Armor(const Armor&)            = delete;
    Armor& operator=(const Armor&) = delete;

    auto update() noexcept -> void;

private:
    auto impl_move(const Translation&, const Orientation&) noexcept -> void;

    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}

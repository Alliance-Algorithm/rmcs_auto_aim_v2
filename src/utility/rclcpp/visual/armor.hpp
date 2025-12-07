#pragma once
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/visual/movable.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace rmcs::util::visual {

struct Armor : public Movable {
public:
    struct Config {
        RclcppNode& rclcpp;

        DeviceId device;
        CampColor camp;

        std::string id;
        std::string tf;
    };

    explicit Armor(const Config&) noexcept;
    explicit Armor(Config const&,
        std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> const&) noexcept;
    ~Armor() noexcept;

    Armor(const Armor&)            = delete;
    Armor& operator=(const Armor&) = delete;

    auto update() noexcept -> void;

    auto impl_move(const Translation&, const Orientation&) noexcept -> void;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}

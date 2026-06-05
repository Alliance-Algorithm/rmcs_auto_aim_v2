#pragma once
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/visual/movable.hpp"
#include "utility/robot/armor.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

#include <span>

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

    auto set_camp(CampColor camp) noexcept -> void;
    auto update() noexcept -> void;

private:
    auto impl_move(const Translation&, const Orientation&) noexcept -> void;

    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

struct Armors {
public:
    struct Config {
        RclcppNode& rclcpp;

        std::string name;
        std::string tf;
    };

    explicit Armors(Config) noexcept;

    ~Armors() noexcept;

    Armors(const Armors&)            = delete;
    Armors& operator=(const Armors&) = delete;
    Armors(Armors&&) noexcept;
    Armors& operator=(Armors&&) noexcept;

    auto update(std::span<const Armor3d> armors) noexcept -> void;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}

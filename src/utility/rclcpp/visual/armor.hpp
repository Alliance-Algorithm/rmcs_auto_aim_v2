#pragma once
#include "utility/linear.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::util::visual {

struct Armor {
    struct Config {
        RclcppNode& rclcpp;

        DeviceId device;
        CampColor camp;

        std::string id;
        std::string tf;
    };

    explicit Armor(const Config&) noexcept;
    ~Armor() noexcept;

    Armor(const Armor&)            = delete;
    Armor& operator=(const Armor&) = delete;

    auto update() noexcept -> void;

    auto move(const translation_trait auto& t, const orientation_trait auto& q) noexcept {
        this->move(Translation { t }, Orientation { q });
    }
    auto move(const std::tuple<Translation, Orientation>& tuple) noexcept {
        this->move(std::get<0>(tuple), std::get<1>(tuple));
    }
    auto move(const Translation&, const Orientation&) noexcept -> void;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}

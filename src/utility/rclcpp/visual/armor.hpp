#pragma once
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/visual/movable.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::util::visual {

struct Armor : public Movable {
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

    /**
 * @brief 禁止拷贝构造以防止 Armor 实例被复制及其内部实现被共享。
 */
Armor(const Armor&)            = delete;
    Armor& operator=(const Armor&) = delete;

    auto update() noexcept -> void;

    auto impl_move(const Translation&, const Orientation&) noexcept -> void;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}
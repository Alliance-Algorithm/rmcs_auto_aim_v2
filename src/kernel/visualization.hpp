#pragma once
#include "utility/image/image.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/robot/armor.hpp"

#include <expected>
#include <yaml-cpp/yaml.h>

namespace rmcs::kernel {

class Visualization {
    RMCS_PIMPL_DEFINITION(Visualization)

public:
    static constexpr auto get_prefix() noexcept { return "visualization"; }

    /**
     * @brief 将图像发送到可视化通道并返回当前 Visualization 的引用。
     *
     * 将给定的 Image 发送或渲染到 Visualization 管理的可视化管道或输出。
     *
     * @param image 要发送或渲染的图像。
     * @return Visualization& 当前对象的引用，便于链式调用。
     */
    auto operator<<(const Image& image) noexcept -> Visualization& {
        return send_image(image), *this;
    }

public:
    auto initialize(const YAML::Node& yaml, util::RclcppNode& visual_node) noexcept
        -> std::expected<void, std::string>;

    auto initialized() const noexcept -> bool;

    auto send_image(const Image&) noexcept -> bool;

    auto visualize_armors(std::span<Armor3D> const& armors) const -> bool;
};

}
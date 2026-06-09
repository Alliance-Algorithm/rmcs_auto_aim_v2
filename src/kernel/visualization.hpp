#pragma once

#include <expected>
#include <span>
#include <string>

#include <yaml-cpp/yaml.h>

#include "utility/image/drawable.hpp"
#include "utility/image/image.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::kernel {

class Visualization {
    RMCS_PIMPL_DEFINITION(Visualization)

public:
    static constexpr auto get_prefix() noexcept { return "visualization"; }

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string>;

    auto initialized() const noexcept -> bool;

    auto update_image(Image& image) -> bool;

    /// Publishable

    auto publish(const Armor3d& armor, const std::string& name) -> void {
        publish(std::span<const Armor3d> { &armor, 1 }, name);
    }
    auto publish(std::span<const Armor3d> armors, const std::string& name) -> void;

    auto publish(const Lightbar3d& lightbar, const std::string& name) -> void {
        publish(std::span<const Lightbar3d> { &lightbar, 1 }, name);
    }
    auto publish(std::span<const Lightbar3d> lightbars, const std::string& name) -> void;

    auto publish_odom(const Transform& t, const std::string& name) -> void;

    auto update_aiming_direction(double yaw, double pitch) const -> void;

    auto update_mpc_plan(double yaw, double pitch, double yaw_rate, double pitch_rate,
        double yaw_acc, double pitch_acc) const -> void;

    /// Drawable

    template <drawable_trait T>
    auto draw_later(const T& drawable) {
        draw_later(std::make_unique<Drawable<T>>(drawable));
    }
    template <drawable_trait T>
    auto draw_later(const std::optional<T>& drawable) {
        if (drawable) {
            draw_later(*drawable);
        }
    }
    template <drawable_trait T>
    auto draw_later(const std::vector<T>& items) {
        for (const auto& item : items)
            draw_later(std::make_unique<Drawable<T>>(item));
    }

private:
    auto draw_later(std::unique_ptr<IDrawable>) -> void;
};

} // namespace rmcs::kernel

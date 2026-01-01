#include "armor_visualizer.hpp"

#include "utility/rclcpp/visual/armor.hpp"
#include "utility/robot/armor.hpp"

using namespace rmcs::debug;
using VisualArmor = rmcs::util::visual::Armor;

struct ArmorShadow {
    decltype(rmcs::Armor3D::genre) genre;
    decltype(rmcs::Armor3D::color) color;
    decltype(rmcs::Armor3D::id) id;

    bool operator==(ArmorShadow const& other) const = default;
    bool operator!=(ArmorShadow const& other) const { return !(*this == other); }
};

struct ArmorVisualizer::Impl final {
    auto initialize(util::RclcppNode& visual_node) noexcept -> void {
        node = std::ref(visual_node);
    }

    auto visualize(std::span<Armor3D const> _armors) -> bool {
        if (!node.has_value()) {
            return false;
        }

        auto new_size = _armors.size();
        visual_armors.reserve(new_size);
        current_armors.reserve(new_size);
        visual_armors.resize(new_size);
        current_armors.resize(new_size);

        for (size_t i = 0; i < new_size; i++) {
            auto const& input = _armors[i];
            auto& armor_ptr   = visual_armors[i];
            auto& shadow      = current_armors[i];

            bool changed = !armor_ptr || needs_rebuild(shadow, input);

            if (changed) {
                auto const config = VisualArmor::Config {
                    .rclcpp = node.value().get(),
                    .device = input.genre,
                    .camp   = armor_color2camp_color(input.color),
                    .id     = input.id,
                    .name   = "solved_pnp_armor",
                    .tf     = "camera_link",
                };

                armor_ptr = std::make_unique<VisualArmor>(config);

                shadow.genre = input.genre;
                shadow.color = input.color;
                shadow.id    = input.id;
            }

            armor_ptr->move(input.translation, input.orientation);
            armor_ptr->update();
        }

        return true;
    }

    // static auto camp(ArmorColor const& color) -> CampColor {
    //     if (color == ArmorColor::BLUE) return CampColor::BLUE;
    //     if (color == ArmorColor::RED) return CampColor::RED;
    //     return CampColor::UNKNOWN;
    // };

    static bool needs_rebuild(ArmorShadow shadow, Armor3D const& input) {
        return shadow.genre != input.genre || shadow.color != input.color || shadow.id != input.id;
    }

    std::optional<std::reference_wrapper<util::RclcppNode>> node;
    std::vector<ArmorShadow> current_armors;
    std::vector<std::unique_ptr<VisualArmor>> visual_armors;
};

auto ArmorVisualizer::initialize(util::RclcppNode& visual_node) noexcept -> void {
    return pimpl->initialize(visual_node);
}

auto ArmorVisualizer::visualize(std::span<Armor3D const> armors) -> bool {
    return pimpl->visualize(armors);
}

ArmorVisualizer::ArmorVisualizer() noexcept
    : pimpl { std::make_unique<Impl>() } { };
ArmorVisualizer::~ArmorVisualizer() noexcept = default;

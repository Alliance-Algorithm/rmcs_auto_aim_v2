#include "armor_visualizer.hpp"

#include "utility/rclcpp/visual/armor.hpp"
#include "utility/robot/armor.hpp"

using namespace rmcs::debug;
using VisualArmor = rmcs::util::visual::Armor;

struct ArmorShadow {
    decltype(rmcs::Armor3D::genre) genre;
    decltype(rmcs::Armor3D::color) color;
    decltype(rmcs::Armor3D::id) id;
    std::string ns;

    bool operator==(ArmorShadow const& other) const = default;
    bool operator!=(ArmorShadow const& other) const { return !(*this == other); }
};

struct ArmorVisualizer::Impl final {
    auto initialize(util::RclcppNode& visual_node) noexcept -> void {
        node = std::ref(visual_node);
    }

    auto visualize(std::span<Armor3D const> _armors, std::string const& name,
        std::string const& link_name) -> bool {
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

            bool changed = !armor_ptr || needs_rebuild(shadow, input, name);

            if (changed) {
                auto const config = VisualArmor::Config {
                    .rclcpp = node.value().get(),
                    .device = input.genre,
                    .camp   = armor_color2camp_color(input.color),
                    .id     = input.id,
                    .name   = name,
                    .tf     = link_name,
                };

                armor_ptr = std::make_unique<VisualArmor>(config);

                shadow.genre = input.genre;
                shadow.color = input.color;
                shadow.id    = input.id;
                shadow.ns    = name;
            }

            armor_ptr->move(input.translation, input.orientation);
            armor_ptr->update();
        }

        return true;
    }

    static auto needs_rebuild(
        ArmorShadow const& shadow, Armor3D const& input, std::string_view name) -> bool {
        return shadow.genre != input.genre || shadow.color != input.color || shadow.id != input.id
            || shadow.ns != name;
    }

    std::optional<std::reference_wrapper<util::RclcppNode>> node;
    std::vector<ArmorShadow> current_armors;
    std::vector<std::unique_ptr<VisualArmor>> visual_armors;
};

auto ArmorVisualizer::initialize(util::RclcppNode& visual_node) noexcept -> void {
    return pimpl->initialize(visual_node);
}

auto ArmorVisualizer::visualize(std::span<Armor3D const> armors, std::string const& name,
    std::string const& link_name) -> bool {
    return pimpl->visualize(armors, name, link_name);
}

ArmorVisualizer::ArmorVisualizer() noexcept
    : pimpl { std::make_unique<Impl>() } { };
ArmorVisualizer::~ArmorVisualizer() noexcept = default;

#pragma once
#include "utility/details.hpp"
#include "utility/linear.hpp"
#include "utility/panic.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

#include <algorithm>
#include <ranges>

namespace rmcs::util {

namespace visual {

    namespace details {
        constexpr auto check_naming(std::string_view name) noexcept -> bool {
            return std::ranges::all_of(name, [](char c) {
                return (c >= 'a' && c <= 'z') || (c >= '0' && c < '9') || (c == '_');
            }) && !std::ranges::empty(name);
        }
        constexpr auto naming_standard = "Names must match pattern: ^[a-z0-9_]+$";

        struct Context {
            RMCS_DETAILS_DEFINITION(Context)

            std::string id;
            std::string tf;

            auto update() noexcept -> void;

            auto share_rclcpp_context(Context&) const noexcept -> void;
        };

        template <class T>
        concept visual_trait = std::same_as<Context, decltype(std::declval<T>().context)>;
    }

    struct Armor {
        details::Context context;

        DeviceId device_id;
        CampColor camp_color;

        Armor() noexcept = default;

        auto init(DeviceId _device_id, CampColor _camp_color) noexcept {
            device_id  = _device_id;
            camp_color = _camp_color;
            init_context();
        }

        auto update() noexcept -> void;

        auto set_translation(const translation_trait auto& t) noexcept -> void {
            set_translation(Translation { t });
        }
        auto set_orientation(const orientation_trait auto& o) noexcept -> void {
            set_orientation(Orientation { o });
        }

        auto set_translation(const Translation&) noexcept -> void;
        auto set_orientation(const Orientation&) noexcept -> void;

    private:
        auto init_context() noexcept -> void;
    };
    static_assert(details::visual_trait<Armor>);

    struct AssembledArmors {
        details::Context context;

        std::array<Armor, 4> armors;
        double w, h;

        Translation translation;
        Orientation orientation;

        auto init(DeviceId device_id, CampColor camp_color, double w, double h) noexcept -> void;

        auto set_translation(const translation_trait auto& t) noexcept { translation = t; }
        auto set_orientation(const orientation_trait auto& o) noexcept { orientation = o; }

        auto update() noexcept -> void;
    };
    static_assert(details::visual_trait<AssembledArmors>);
}

class VisualNode {
    RMCS_PIMPL_DEFINITION(VisualNode)

public:
    explicit VisualNode(const std::string& id) noexcept;

    template <visual::details::visual_trait T>
    auto make(const std::string& id, const std::string& tf) noexcept -> std::unique_ptr<T> {
        auto item = std::make_unique<T>();

        if (!visual::details::check_naming(id)) {
            util::panic(std::format(
                "Not a valid naming for armor id: {}", visual::details::naming_standard));
        }
        if (!visual::details::check_naming(tf)) {
            util::panic(std::format(
                "Not a valid naming for armor tf: {}", visual::details::naming_standard));
        }

        item->context.id = id;
        item->context.tf = tf;

        bind_context(item->context);
        return item;
    }

private:
    auto bind_context(visual::details::Context& context) noexcept -> void;
};
}

#pragma once
#include "utility/details.hpp"
#include "utility/linear.hpp"
#include "utility/panic.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::util {

namespace visual {

    struct Context {
        RMCS_DETAILS_DEFINITION(Context)

        std::string topic_name;
        std::string tf_name;

        auto update() noexcept -> void;
    };
    template <class T>
    concept visual_trait = requires(T t) {
        { t.context } -> std::same_as<Context>;
    };

    struct Armor {
        DeviceId device_id;
        CampColor camp_color;

        Context context;

        explicit Armor(const std::string& id, const std::string& tf) noexcept {
            if (id.contains('/')) {
                util::panic(std::format("Invalid id containing '/': {}", id));
            }
            context.topic_name = "/rmcs/auto_aim/armor/" + id;
            context.tf_name    = tf;
        }

        auto update() noexcept -> void;

        auto set_translation(const Translation&) noexcept -> void;
        auto set_orientation(const Orientation&) noexcept -> void;
    };
}

class Visualization {
    RMCS_PIMPL_DEFINITION(Visualization)

public:
    explicit Visualization(const std::string& id) noexcept;

    auto set_topic_prefix(const std::string&) noexcept -> void;

    template <visual::visual_trait T>
    auto make_visualized() noexcept -> std::unique_ptr<T> {
        auto item = std::make_unique<visual::Armor>("", "");
        bind_context(item->context);
        return item;
    }

private:
    auto bind_context(visual::Context& context) noexcept -> void;
};

}

#pragma once
#include "utility/image.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

namespace rmcs::kernel {

namespace identifier::details {
    using ImageUnique = std::unique_ptr<Image>;

    template <class T>
    concept fetchable_trait = requires(T& t) {
        { t.fetch() } -> std::same_as<ImageUnique>;
    };
    template <fetchable_trait T>
    struct CaptureAdapter final {
        T& fetchable;

        explicit CaptureAdapter(T& capturer) noexcept
            : fetchable { capturer } { }

        auto fetch() const noexcept -> ImageUnique { return fetchable.fetch(); }
    };

    struct IdentifierImpl final {
        RMCS_PIMPL_DEFINITION(IdentifierImpl)

    public:
        using IdentifyResult = void;
        using ResultUnique   = std::unique_ptr<IdentifyResult>;

        auto push_source(ImageUnique source) noexcept -> void;
        auto pull_result() noexcept -> ResultUnique;

        auto set_enemy_color(CampColor color) noexcept -> void;
        auto set_enemy_target(DeviceIds ids) noexcept -> void;
    };
};

}

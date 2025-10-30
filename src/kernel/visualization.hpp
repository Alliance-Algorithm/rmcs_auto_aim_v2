#pragma once
#include "utility/image.hpp"
#include "visualization.config.hpp"

namespace rmcs::kernel {

class VisualizationRuntime {
    RMCS_PIMPL_DEFINITION(VisualizationRuntime)

public:
    static constexpr auto get_prefix() noexcept { return "visualization"; }

    auto operator<<(const Image& image) noexcept -> VisualizationRuntime& {
        return send_image(image), *this;
    }

public:
    using Config = VisualizationConfig;
    auto initialize(const Config&) noexcept -> std::expected<void, std::string>;

    auto send_image(const Image&) noexcept -> bool;
};

}

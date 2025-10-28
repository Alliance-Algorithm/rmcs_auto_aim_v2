#pragma once
#include "modules/capturer/common.hpp"
#include "utility/image.impl.hpp"
#include <hikcamera/capturer.hpp>

namespace rmcs::cap::details {

struct HikcameraImpl {
    using Config = hikcamera::Config;

    hikcamera::Camera details;

    explicit HikcameraImpl() noexcept
        : details {} { }

    auto initialize(const Config& config) noexcept -> NormalResult {
        return details.initialize(config);
    }

    auto deinitialize() noexcept -> NormalResult { return details.deinitialize(); }

    auto initialized() const noexcept -> bool { return details.initialized(); }

    auto wait_image() noexcept -> ImageResult {
        auto mat = details.read_image();
        if (!mat.has_value()) {
            return std::unexpected { mat.error() };
        }

        auto image = std::make_unique<Image>();
        image->details().set_mat(std::move(*mat));

        return image;
    }
};

}
namespace rmcs::cap {

using Hikcamera = cap::Adapter<details::HikcameraImpl>;

}

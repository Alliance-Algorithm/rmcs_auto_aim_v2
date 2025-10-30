#pragma once
#include "utility/image.hpp"

#include <expected>
#include <string>

namespace rmcs::cap {

using NormalResult = std::expected<void, std::string>;
using ImageResult  = std::expected<std::unique_ptr<Image>, std::string>;

template <class Impl>
struct Adapter : public Impl {
public:
    using Config = Impl::Config;
    using Impl::Impl;

    auto wait_image() noexcept -> ImageResult { return Impl::wait_image(); }

    auto initialize(const Config& config = {}) noexcept -> NormalResult {
        return Impl::initialize(config);
    }

    auto deinitialize() noexcept -> NormalResult { return Impl::deinitialize(); }

    auto initialized() const noexcept -> bool { return Impl::initialized(); }

    static constexpr auto get_prefix() noexcept { return Impl::get_prefix(); }

private:
    static constexpr auto has_config = requires { typename Impl::Config; };
    static_assert(has_config, "Capturer must has Config type");

    static constexpr auto has_wait_image = requires(Impl& i) {
        { i.wait_image() } -> std::same_as<ImageResult>;
    };
    static_assert(has_wait_image, "Capturer must has wait image function");

    static constexpr auto has_initialize = requires(Impl& i) {
        { i.initialized() } -> std::same_as<bool>;
        { i.initialize(Config {}) } -> std::same_as<NormalResult>;
        { i.deinitialize() } -> std::same_as<NormalResult>;
    };
    static_assert(has_initialize, "Capturer must has initialize functions");

    static constexpr auto has_get_prefix = requires(Impl& i) {
        { i.get_prefix() } -> std::assignable_from<std::string>;
    };
    static_assert(has_wait_image, "Capturer must has get prefix function");
};

}

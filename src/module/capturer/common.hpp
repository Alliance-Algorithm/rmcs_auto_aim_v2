#pragma once
#include "utility/image/image.hpp"

#include <expected>
#include <yaml-cpp/yaml.h>

namespace rmcs::cap {

using NormalResult = std::expected<void, std::string>;
using ImageResult  = std::expected<std::unique_ptr<Image>, std::string>;

using Yaml = YAML::Node;

struct Interface {
    virtual auto wait_image() noexcept -> ImageResult {
        return std::unexpected { "Unimplement interface: 'wait'" };
    }
    virtual auto connect() noexcept -> NormalResult {
        return std::unexpected { "Unimplement interface: 'connect'" };
    }
    virtual auto disconnect() noexcept -> void { }
    virtual auto connected() const noexcept -> bool { return false; }

    virtual ~Interface() noexcept = default;
};

template <class Impl>
struct Adapter : public Impl, Interface {
public:
    using Impl::Impl;
    using Config = Impl::Config;

    auto configure_yaml(const Yaml& yaml) noexcept -> NormalResult {
        auto config = Config {};
        auto result = config.serialize(yaml);

        if (result.has_value()) {
            Impl::configure(config);
            return {};
        } else {
            return std::unexpected { result.error() };
        }
    }

    static_assert(requires(Impl& impl) { impl.wait_image(); }, "实现必须拥有 wait_image 函数");
    auto wait_image() noexcept -> ImageResult override { return Impl::wait_image(); }

    static_assert(requires(Impl& impl) { impl.connect(); }, "实现必须拥有 connect 函数");
    auto connect() noexcept -> NormalResult override { return Impl::connect(); }

    static_assert(requires(Impl& impl) { impl.disconnect(); }, "实现必须拥有 disconnect 函数");
    auto disconnect() noexcept -> void override { Impl::disconnect(); }

    static_assert(
        requires(Impl& impl) { bool { impl.connected() }; }, "实现必须拥有 connected 函数");
    auto connected() const noexcept -> bool override { return Impl::connected(); }
};

}

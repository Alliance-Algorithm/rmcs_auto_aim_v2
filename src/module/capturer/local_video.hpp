#pragma once
#include "utility/image/image.hpp"
#include "utility/serializable.hpp"
#include <expected>
#include <yaml-cpp/yaml.h>

namespace rmcs::cap {

class LocalVideo {
    RMCS_PIMPL_DEFINITION(LocalVideo)

public:
    struct ConfigDetail {
        std::string location;
    };

    struct Config : ConfigDetail, util::Serializable {
        constexpr static std::tuple metas {
            &Config::location,
            "location",
        };
    };

    auto configure(const ConfigDetail&) noexcept -> std::expected<void, std::string>;

    auto connect() noexcept -> std::expected<void, std::string>;

    auto connected() const noexcept -> bool;

    auto wait_image() -> std::expected<std::unique_ptr<Image>, std::string>;
};

}

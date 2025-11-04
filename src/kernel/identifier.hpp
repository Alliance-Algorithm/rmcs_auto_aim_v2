#pragma once
#include "identifier.config.hpp"

#include "utility/image.hpp"
#include "utility/pimpl.hpp"

namespace rmcs::runtime {

class Identifier {
    RMCS_PIMPL_DEFINITION(Identifier)

public:
    using Config = IdentifierConfig;

    auto initialize(const Config&) noexcept -> std::expected<void, std::string>;

    auto perview() const noexcept -> const Image&;

    auto sync_identify(const Image&) noexcept -> void;

    static constexpr auto get_prefix() { return "identifier"; }
};

}

#pragma once

#include "utility/image/image.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

#include <expected>
#include <yaml-cpp/node/node.h>

namespace rmcs::kernel {

class Identifier {
    RMCS_PIMPL_DEFINITION(Identifier)

public:
    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto sync_identify(const Image&) noexcept -> std::optional<std::vector<Armor2D>>;
};

}

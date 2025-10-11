#pragma once
#include "utility/image.hpp"
#include "utility/pimpl.hpp"
#include <expected>
#include <yaml-cpp/node/node.h>

namespace rmcs::kernel {

class Capturer final {
    RMCS_PIMPL_DEFINITION(Capturer)

public:
    auto initialize() noexcept -> std::expected<void, std::string>;

    auto start_working() noexcept -> void;

    auto stop_working() noexcept -> void;

    /// Safe for multi-thread
    auto fetch() noexcept -> std::unique_ptr<Image>;
};

}

#pragma once
#include "utility/image.hpp"
#include "utility/pimpl.hpp"
#include <expected>
#include <yaml-cpp/node/node.h>

namespace rmcs::kernel {

class Capturer final {
    RMCS_PIMPL_DEFINITION(Capturer)

public:
    /// Init with config yaml
    auto initialize() noexcept -> std::expected<void, std::string>;

    /// Start working thread
    auto start_working() noexcept -> void;

    /// Stop working thread
    auto stop_working() noexcept -> void;

    /// Safe for multi-thread
    auto fetch() noexcept -> std::unique_ptr<Image>;
};

}

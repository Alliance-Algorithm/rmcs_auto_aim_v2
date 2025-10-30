#pragma once
#include "utility/image.hpp"
#include "utility/pimpl.hpp"
#include <expected>

namespace rmcs::kernel {

class CapRuntime final {
    RMCS_PIMPL_DEFINITION(CapRuntime)

public:
    using ImageUnique = std::unique_ptr<Image>;

    auto initialize() noexcept -> std::expected<void, std::string>;

    /// @brief
    ///   Fetches an image from the background worker thread.
    /// @note
    ///   - Non-blocking: returns immediately with either a valid image or nullptr.
    ///   - Thread-safe: safe to call from multiple threads, but only one thread
    ///     should fetch at a time.
    auto fetch_image() noexcept -> ImageUnique;
};

}

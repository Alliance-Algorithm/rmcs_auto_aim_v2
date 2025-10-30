#pragma once
#include "capturer.config.hpp"
#include "utility/image.hpp"
#include "utility/pimpl.hpp"

#include <expected>

namespace rmcs::kernel {

class Capturer final {
    RMCS_PIMPL_DEFINITION(Capturer)

public:
    using ImageUnique = std::unique_ptr<Image>;

    static constexpr auto get_prefix() noexcept { return "capturer"; }

    using Config = CapturerConfig;
    auto initialize(const Config&) noexcept -> std::expected<void, std::string>;

    /// @brief
    ///   Fetches an image from the background worker thread.
    /// @note
    ///   - Non-blocking: returns immediately with either a valid image or nullptr.
    ///   - Thread-safe: safe to call from multiple threads, but only one thread
    ///     should fetch at a time.
    auto fetch_image() noexcept -> ImageUnique;
};

}

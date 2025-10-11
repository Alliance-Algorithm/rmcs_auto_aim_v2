#pragma once
#include "utility/pimpl.hpp"

#include <expected>
#include <hikcamera/image_capturer.hpp>

namespace rmcs::capturer {

class HikcameraCap final {
    RMCS_PIMPL_DEFINITION(HikcameraCap)

public:
    using ms = std::chrono::milliseconds;

    using Capture = hikcamera::ImageCapturer;
    using Profile = Capture::CameraProfile;

    auto initialize(Profile const& profile = {}) noexcept -> std::expected<void, std::string_view>;

    auto initialized() const noexcept -> bool;

    auto set_frame_rate_inner_trigger_mode(std::float_t frame_rate) noexcept -> void;

    auto reset() noexcept -> void;

    auto read(ms timeout = ms { 1 }) noexcept -> std::expected<cv::Mat, std::string_view>;

    /// @throw std::runtime_error Failed to read image
    auto read_with_exception(ms timeout = ms { 1 }) -> cv::Mat;
};

}

#include "hikcamera.hpp"

using namespace rmcs::capturer;

struct HikcameraCap::Impl {
    std::unique_ptr<Capture> hikcamera = nullptr;

    auto initialize(Profile const& profile) noexcept -> std::expected<void, std::string_view> {
        try {
            hikcamera = std::make_unique<Capture>(profile);
        } catch (std::runtime_error const& e) {
            return std::unexpected { e.what() };
        }
        return {};
    }

    auto initialized() const noexcept { return bool { hikcamera }; }

    auto set_frame_rate_inner_trigger_mode(std::float_t frame_rate) const noexcept {
        hikcamera->set_frame_rate_inner_trigger_mode(frame_rate);
    }

    auto reset() noexcept { hikcamera.reset(); }

    auto read(std::chrono::milliseconds timeout) const noexcept
        -> std::expected<cv::Mat, std::string_view> {
        try {
            return hikcamera->read(timeout);
        } catch (std::runtime_error const& error) {
            return std::unexpected { error.what() };
        }
    }

    auto read_with_exception(std::chrono::milliseconds timeout) const {
        return hikcamera->read(timeout);
    }
};

HikcameraCap::HikcameraCap() noexcept
    : pimpl { std::make_unique<Impl>() } { }

HikcameraCap::~HikcameraCap() noexcept = default;

auto HikcameraCap::initialize(Profile const& profile) noexcept
    -> std::expected<void, std::string_view> {
    return pimpl->initialize(profile);
}

auto HikcameraCap::initialized() const noexcept -> bool { return pimpl->initialized(); }

auto HikcameraCap::set_frame_rate_inner_trigger_mode(std::float_t frame_rate) noexcept -> void {
    pimpl->set_frame_rate_inner_trigger_mode(frame_rate);
}

auto HikcameraCap::reset() noexcept -> void { pimpl->reset(); }

auto HikcameraCap::read(std::chrono::milliseconds timeout) noexcept
    -> std::expected<cv::Mat, std::string_view> {
    return pimpl->read(timeout);
}

auto HikcameraCap::read_with_exception(std::chrono::milliseconds timeout) -> cv::Mat {
    return pimpl->read_with_exception(timeout);
}

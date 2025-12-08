#include "local_video.hpp"
#include "utility/image/image.details.hpp"

#include <filesystem>
#include <opencv2/videoio.hpp>

using namespace rmcs::cap;
using rmcs::Image;

struct LocalVideo::Impl {
    cv::VideoCapture cap;
    ConfigDetail config;

    auto release() noexcept {
        if (cap.isOpened()) cap.release();
    }
};

auto LocalVideo::configure(const ConfigDetail& config) noexcept
    -> std::expected<void, std::string> {
    pimpl->config = config;

    if (pimpl->config.location.empty()) {
        return std::unexpected { "Video location is empty" };
    }
    if (!std::filesystem::exists(pimpl->config.location)) {
        return std::unexpected { "Video file not found: " + pimpl->config.location };
    }
    return {};
}

auto LocalVideo::connect() noexcept -> std::expected<void, std::string> {
    pimpl->release();

    if (!pimpl->cap.open(pimpl->config.location)) {
        return std::unexpected { "Failed to open video: " + pimpl->config.location };
    }
    return {};
}

auto LocalVideo::connected() const noexcept -> bool { return pimpl->cap.isOpened(); }

auto LocalVideo::wait_image() -> std::expected<std::unique_ptr<Image>, std::string> {
    if (!connected()) {
        return std::unexpected { "Video is not opened" };
    }

    cv::Mat frame;
    if (!pimpl->cap.read(frame) || frame.empty()) {
        if (pimpl->config.loop_play) {
            pimpl->cap.set(cv::CAP_PROP_POS_FRAMES, 0);
            if (pimpl->cap.read(frame) && !frame.empty()) {
            } else {
                return std::unexpected { "Failed to read frame even after seeking to beginning" };
            }
        }
        return std::unexpected { "Failed to read frame" };
    }

    auto image = std::make_unique<Image>();
    image->details().set_mat(std::move(frame));
    image->set_timestamp(Image::Clock::now());
    return image;
}

auto LocalVideo::disconnect() noexcept -> void { pimpl->release(); }

LocalVideo::LocalVideo() noexcept
    : pimpl { std::make_unique<Impl>() } { }

LocalVideo::~LocalVideo() noexcept = default;

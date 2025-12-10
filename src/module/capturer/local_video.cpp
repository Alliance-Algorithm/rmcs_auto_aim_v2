#include "local_video.hpp"

#include <filesystem>
#include <thread>

#include <opencv2/videoio.hpp>

#include "utility/image/image.details.hpp"

using namespace rmcs::cap;

struct LocalVideo::Impl {
    Config config;

    using Clock = std::chrono::steady_clock;

    std::optional<cv::VideoCapture> capturer;

    std::chrono::nanoseconds interval_duration { 0 };
    Clock::time_point last_read_time { Clock::now() };

    auto set_framerate_interval(double hz) noexcept -> void {
        if (hz > 0) {
            interval_duration =
                std::chrono::nanoseconds(static_cast<long long>(std::round(1.0 / hz * 1e9)));
        } else {
            interval_duration = std::chrono::nanoseconds { 0 };
        }
    };

    auto configure(Config const& _config) -> std::expected<void, std::string> {
        if (_config.location.empty() || !std::filesystem::exists(_config.location)) {
            return std::unexpected { "Local video is not found or location is empty" };
        }

        config = _config;

        try {
            capturer.emplace(config.location);
        } catch (std::exception const& e) {
            return std::unexpected { "Failed to construct VideoCapture: " + std::string(e.what()) };
        } catch (...) {
            return std::unexpected { "Failed to construct VideoCapture due to an unknown error." };
        }

        double source_fps = capturer->get(cv::CAP_PROP_FPS);
        double target_fps = source_fps > 0 ? source_fps : 30.0;

        if (config.frame_rate > 0) {
            target_fps = config.frame_rate;
        }

        set_framerate_interval(target_fps);

        last_read_time = Clock::now();

        return {};
    }

    auto connect() -> std::expected<void, std::string> { return configure(config); }

    auto connected() const noexcept -> bool { return capturer.has_value() && capturer->isOpened(); }

    auto disconnect() noexcept -> std::expected<void, std::string> {
        if (capturer.has_value()) {
            capturer.reset();
        }
        interval_duration = std::chrono::nanoseconds { 0 };

        return {};
    }

    auto wait_image() noexcept -> std::expected<std::unique_ptr<Image>, std::string> {
        if (!capturer.has_value() || !capturer->isOpened()) {
            return std::unexpected { "Video stream is not opened." };
        }

        const auto time_before_read        = Clock::now();
        const auto next_read_time_expected = last_read_time + interval_duration;
        auto wait_duration                 = next_read_time_expected - time_before_read;

        if (wait_duration.count() > 0) {
            std::this_thread::sleep_for(wait_duration);
            last_read_time = next_read_time_expected;
        } else {
            last_read_time = config.allow_skipping ? Clock::now() : next_read_time_expected;
        }

        auto frame = cv::Mat {};
        auto image = std::make_unique<Image>();
        if (!capturer->read(frame)) {
            if (config.loop_play) {
                if (capturer->set(cv::CAP_PROP_POS_FRAMES, 0) && capturer->read(frame)) {
                    last_read_time = Clock::now();
                } else {
                    return std::unexpected { "End of file reached and failed to "
                                             "loop/reset." };
                }
            } else {
                return std::unexpected { "End of file reached." };
            }
        }

        if (frame.empty()) {
            return std::unexpected { "Read frame is empty, possibly due to IO error." };
        }
        image->details().set_mat(frame);
        image->set_timestamp(last_read_time);

        return image;
    };
};

auto LocalVideo::configure(Config const& config) -> std::expected<void, std::string> {
    return pimpl->configure(config);
}
auto LocalVideo::wait_image() noexcept -> std::expected<std::unique_ptr<Image>, std::string> {
    return pimpl->wait_image();
}

auto LocalVideo::connect() noexcept -> std::expected<void, std::string> { return pimpl->connect(); }

auto LocalVideo::connected() const noexcept -> bool { return pimpl->connected(); }

auto LocalVideo::disconnect() noexcept -> std::expected<void, std::string> {
    return pimpl->disconnect();
}

LocalVideo::LocalVideo() noexcept
    : pimpl { std::make_unique<Impl>() } { }

LocalVideo::~LocalVideo() noexcept = default;

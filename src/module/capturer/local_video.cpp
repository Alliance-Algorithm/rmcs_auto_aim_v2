#include "local_video.hpp"
#include "utility/service.hpp"
#include "utility/singleton/running.hpp"

#include <atomic>
#include <filesystem>
#include <mutex>
#include <optional>
#include <thread>

#include <opencv2/videoio.hpp>

using namespace rmcs::cap;

struct LocalVideo::Impl {
    using Clock     = std::chrono::steady_clock;
    using TimePoint = std::chrono::steady_clock::time_point;

    Config config;

    std::optional<cv::VideoCapture> capturer;
    std::mutex image_mutex;
    bool playing { true };
    std::optional<Image> latest_image;

    std::chrono::nanoseconds interval_duration { 0 };
    TimePoint last_read_time { Clock::now() };

    std::jthread service_thread;

    auto set_framerate_interval(double hz) noexcept -> void {
        if (hz > 0) {
            interval_duration =
                std::chrono::nanoseconds(static_cast<long long>(std::round(1.0 / hz * 1e9)));
        } else {
            interval_duration = std::chrono::nanoseconds { 0 };
        }
    };

    auto step_frame_locked(int offset) -> void {
        if (!capturer.has_value()) return;

        auto frame = cv::Mat { };
        if (offset < 0) {
            const auto pos = static_cast<int>(capturer->get(cv::CAP_PROP_POS_FRAMES));
            if (pos < 2) return;
            capturer->set(cv::CAP_PROP_POS_FRAMES, pos - 2);
            if (!capturer->read(frame) || frame.empty()) return;
        } else {
            if (!capturer->read(frame) || frame.empty()) return;
        }

        latest_image = Image {
            .mat       = std::move(frame),
            .timestamp = Clock::now(),
        };
    }

    auto start_service() -> void {
        service_thread = std::jthread { [this](const std::stop_token& token) {
            using namespace util;
            auto dirty = std::atomic<bool> { true };

            auto service = Service {
                Named<"local_video"> { },
                Action { Named<"play_pause">(),
                    [this, &dirty](std::string_view) {
                        std::lock_guard guard { image_mutex };
                        playing = !playing;
                        dirty.store(true);
                    } },
                Action { Named<"step_forward">(),
                    [this, &dirty](std::string_view) {
                        std::lock_guard guard { image_mutex };
                        if (playing) return;
                        step_frame_locked(+1);
                        dirty.store(true);
                    } },
                Action { Named<"step_backward">(),
                    [this, &dirty](std::string_view) {
                        std::lock_guard guard { image_mutex };
                        if (playing) return;
                        step_frame_locked(-1);
                        dirty.store(true);
                    } },
            };

            while (!token.stop_requested() && get_running()) {
                service.spin();
                if (dirty.exchange(false)) {
                    std::lock_guard guard { image_mutex };
                    service.update_later("playing", playing ? "1" : "0");
                    service.update_later("source", "local_video");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds { 100 });
            }
        } };
    }

    auto stop_service() noexcept -> void {
        service_thread.request_stop();
        if (service_thread.joinable()) {
            service_thread.join();
        }
    }

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

        auto source_fps = capturer->get(cv::CAP_PROP_FPS);
        auto target_fps = source_fps > 0 ? source_fps : 30.0;

        if (config.frame_rate > 0) {
            target_fps = config.frame_rate;
        }

        set_framerate_interval(target_fps);

        {
            std::lock_guard guard { image_mutex };
            playing      = true;
            latest_image = std::nullopt;
        }
        last_read_time = Clock::now();

        start_service();

        return { };
    }

    auto connect() -> std::expected<void, std::string> { return configure(config); }

    auto connected() const noexcept -> bool { return capturer.has_value() && capturer->isOpened(); }

    auto disconnect() noexcept -> void {
        stop_service();

        {
            std::lock_guard guard { image_mutex };
            if (capturer.has_value()) {
                capturer.reset();
            }
            latest_image = std::nullopt;
        }
        interval_duration = std::chrono::nanoseconds { 0 };
    }

    auto wait_image() noexcept -> std::expected<std::unique_ptr<Image>, std::string> {
        if (!capturer.has_value() || !capturer->isOpened()) {
            return std::unexpected { "Video stream is not opened." };
        }

        const auto time_before_read        = Clock::now();
        const auto next_read_time_expected = last_read_time + interval_duration;
        const auto wait_duration           = next_read_time_expected - time_before_read;

        if (wait_duration.count() > 0) {
            std::this_thread::sleep_for(wait_duration);
            last_read_time = next_read_time_expected;
        } else {
            last_read_time = config.allow_skipping ? Clock::now() : next_read_time_expected;
        }

        {
            std::lock_guard guard { image_mutex };
            if (!playing && latest_image) {
                auto image       = std::make_unique<Image>();
                image->mat       = latest_image->mat.clone();
                image->timestamp = latest_image->timestamp;
                return image;
            }
        }

        auto frame = cv::Mat { };
        auto image = std::make_unique<Image>();
        if (!capturer->read(frame)) {
            if (config.loop_play) {
                if (capturer->set(cv::CAP_PROP_POS_FRAMES, 0) && capturer->read(frame)) {
                    last_read_time = Clock::now();
                } else {
                    return std::unexpected { "End of file reached and failed to loop/reset." };
                }
            } else {
                return std::unexpected { "End of file reached." };
            }
        }

        if (frame.empty()) {
            return std::unexpected { "Read frame is empty, possibly due to IO error." };
        }

        image->mat       = std::move(frame);
        image->timestamp = last_read_time;

        {
            std::lock_guard guard { image_mutex };
            latest_image = {
                .mat       = image->mat.clone(),
                .timestamp = last_read_time,
            };
        }

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

auto LocalVideo::disconnect() noexcept -> void { return pimpl->disconnect(); }

LocalVideo::LocalVideo() noexcept
    : pimpl { std::make_unique<Impl>() } { }

LocalVideo::~LocalVideo() noexcept = default;

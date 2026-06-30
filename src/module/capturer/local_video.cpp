#include "local_video.hpp"

#include <filesystem>
#include <optional>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include <opencv2/videoio.hpp>

using namespace rmcs::cap;

struct LocalVideo::Impl {
    struct TerminalRawMode {
        bool enabled { false };
        ::termios old { };

        auto enable() noexcept -> void {
            if (!::isatty(STDIN_FILENO)) {
                return;
            }
            if (::tcgetattr(STDIN_FILENO, &old) != 0) {
                return;
            }

            auto raw = old;
            raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
            raw.c_cc[VMIN]  = 0;
            raw.c_cc[VTIME] = 0;
            enabled         = (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0);
        }

        auto disable() noexcept -> void {
            if (enabled) {
                ::tcsetattr(STDIN_FILENO, TCSANOW, &old);
                enabled = false;
            }
        }

        ~TerminalRawMode() noexcept { disable(); }

        static auto poll_key(std::chrono::milliseconds timeout) noexcept -> std::optional<char> {
            auto readfds = fd_set { };
            FD_ZERO(&readfds);
            FD_SET(STDIN_FILENO, &readfds);

            auto tv = timeval {
                .tv_sec  = static_cast<long>(timeout.count() / 1'000),
                .tv_usec = static_cast<long>((timeout.count() % 1'000) * 1'000),
            };

            if (::select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv) <= 0) {
                return std::nullopt;
            }

            char key = 0;
            if (::read(STDIN_FILENO, &key, 1) != 1) {
                return std::nullopt;
            }

            return key;
        }
    };

    Config config;

    using Clock     = std::chrono::steady_clock;
    using TimePoint = std::chrono::steady_clock::time_point;

    std::optional<cv::VideoCapture> capturer;
    bool playing { true };
    TerminalRawMode terminal_raw_mode;
    std::optional<Image> latest_image;

    std::chrono::nanoseconds interval_duration { 0 };
    TimePoint last_read_time { Clock::now() };

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

        playing      = true;
        latest_image = std::nullopt;
        terminal_raw_mode.enable();
        last_read_time = Clock::now();

        return { };
    }

    auto connect() -> std::expected<void, std::string> { return configure(config); }

    auto connected() const noexcept -> bool { return capturer.has_value() && capturer->isOpened(); }

    auto disconnect() noexcept -> void {
        if (capturer.has_value()) {
            capturer.reset();
        }
        latest_image = std::nullopt;
        terminal_raw_mode.disable();
        interval_duration = std::chrono::nanoseconds { 0 };
    }

    auto wait_image() noexcept -> std::expected<std::unique_ptr<Image>, std::string> {
        if (!capturer.has_value() || !capturer->isOpened()) {
            return std::unexpected { "Video stream is not opened." };
        }

        using namespace std::chrono_literals;

        if (const auto key = TerminalRawMode::poll_key(0ms); key == ' ') {
            playing = !playing;
            if (playing) {
                last_read_time = Clock::now();
            }
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

        if (!playing && latest_image) {
            auto image = std::make_unique<Image>();
            image->mat = latest_image->mat.clone();
            image->timestamp = latest_image->timestamp;
            return image;
        }

        auto frame = cv::Mat { };
        auto image = std::make_unique<Image>();
        if (!capturer->read(frame)) {
            if (config.loop_play) {
                if (capturer->set(cv::CAP_PROP_POS_FRAMES, 0) && capturer->read(frame)) {
                    last_read_time = Clock::now();
                } else {
                    return std::unexpected {
                        "End of file reached and failed to loop/reset.",
                    };
                }
            } else {
                return std::unexpected { "End of file reached." };
            }
        }

        if (frame.empty()) {
            return std::unexpected { "Read frame is empty, possibly due to IO error." };
        }
        image->mat = std::move(frame);
        image->timestamp = last_read_time;
        latest_image = Image {
            .mat       = image->mat.clone(),
            .timestamp = last_read_time,
        };

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

#include "capturer.hpp"
#include "modules/capturer/hikcamera.hpp"
#include "modules/debug/framerate.hpp"
#include "utility/logging/printer.hpp"
#include "utility/thread/spsc_queue.hpp"

#include <rclcpp/utilities.hpp>
#include <thread>

using Cap = rmcs::cap::Hikcamera;

using namespace rmcs::kernel;

struct CapRuntime::Impl {
    using RawImage = Image*;

    Printer log { "CapRuntime" };
    FramerateCounter loss_image_framerate {};

    std::unique_ptr<Cap> capturer;
    Cap::Config config;

    std::chrono::seconds reconnect_wait_seconds { 1 };

    spsc_queue<Image*, 10> capture_queue;
    std::jthread runtime_thread;

    auto initialize() noexcept -> std::expected<void, std::string> {
        using namespace std::chrono_literals;

        loss_image_framerate.set_intetval(10s);

        // Not connect here, to quick launch
        capturer = std::make_unique<Cap>();
        {
            // TODO: Switch yaml configutation
            config.invert_image = false;
            config.timeout_ms   = 1'000;
        }

        runtime_thread = std::jthread {
            [this](auto t) { runtime_task(t); },
        };

        return { /* Successfully initialize */ };
    }

    ~Impl() noexcept {
        runtime_thread.request_stop();
        if (runtime_thread.joinable()) {
            runtime_thread.join();
        }
    }

    auto fetch_image() noexcept -> ImageUnique {
        auto raw = RawImage { nullptr };
        capture_queue.pop(raw);
        return std::unique_ptr<Image> { raw };
    }

    // TODO: 或许图像池更适合这个场景，毕竟同时处于处理中的图像最多就那么几个
    auto runtime_task(const std::stop_token& token) noexcept -> void {
        log.info("[Capturer runtime thread] starts");

        // Success context
        auto success_callback = [&](std::unique_ptr<Image> image) {
            auto newest = image.release();
            if (!capture_queue.push(newest)) {

                // Failed to push, drop the oldest one
                //   or else delete the newest one
                auto oldest = RawImage { nullptr };
                if (capture_queue.pop(oldest)) [[likely]] {
                    auto guard = std::unique_ptr<Image>(oldest);
                    assert(capture_queue.push(newest) && "Failed to push after pop");
                } else [[unlikely]] {
                    auto guard = std::unique_ptr<Image>(newest);
                    log.error("Pop failed when images queue is full");
                }

                // Log the loss framerate
                if (loss_image_framerate.tick()) {
                    if (auto fps = loss_image_framerate.fps()) {
                        log.warn("Loss image framerate: {}hz", fps);
                    }
                }
            }
        };

        // Failed context
        auto capture_failed_count = std::uint8_t { 0 };
        auto capture_failed_limit = std::uint8_t { 3 };

        auto failed_callback = [&](const std::string& msg) {
            auto& limit = capture_failed_limit;
            auto& count = capture_failed_count;
            if (count++ >= limit) {
                count = 0;
                log.error("Failed to capture image {} times", limit);
                log.error("- Newest error: {}", msg);
                log.error("- Reconnect capturer now...");

                std::ignore = capturer->deinitialize();
                rclcpp::sleep_for(reconnect_wait_seconds);
            }
        };

        // Reconnect context
        auto error_times = std::uint8_t { 0 };
        auto error_limit = std::uint8_t { 3 };
        auto error_stop  = bool { false };

        auto reconnect = [&] {
            if (!capturer->initialized()) {
                auto ret = capturer->deinitialize();
            }
            if (auto result = capturer->initialize(config)) {
                log.info("Connect to capturer successfully");
                error_times = 0;
                error_stop  = false;
            } else {
                if (error_times++ < error_limit) {
                    log.error("Failed to reconnect to capturer, retry soon");
                    log.error("- Error: {}", result.error());
                } else if (!error_stop) {
                    error_stop = true;
                    log.error("{} times, stop printing errors", error_times);
                }
            }
            rclcpp::sleep_for(reconnect_wait_seconds);
        };

        for (;;) {
            if (token.stop_requested()) break;
            if (!rclcpp::ok()) break;

            if (!capturer->initialized()) {
                reconnect();
                continue;
            }

            if (auto result = capturer->wait_image()) {
                success_callback(std::move(*result));
            } else {
                failed_callback(result.error());
            }
        }
        log.info("Because the cancellation operation [capturer thread] has ended");
    }
};

auto CapRuntime::initialize() noexcept -> std::expected<void, std::string> {
    return pimpl->initialize();
}

auto CapRuntime::fetch_image() noexcept -> ImageUnique { return pimpl->fetch_image(); }

CapRuntime::CapRuntime() noexcept
    : pimpl { std::make_unique<Impl>() } { }

CapRuntime::~CapRuntime() noexcept = default;

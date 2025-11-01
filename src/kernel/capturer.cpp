#include "capturer.hpp"
#include "modules/capturer/hikcamera.hpp"
#include "modules/debug/framerate.hpp"
#include "utility/logging/printer.hpp"
#include "utility/thread/spsc_queue.hpp"
#include "utility/times_limit.hpp"

#include <rclcpp/utilities.hpp>
#include <thread>

using Cap = rmcs::cap::Hikcamera;

using namespace rmcs::kernel;

struct Capturer::Impl {
    using RawImage = Image*;

    Printer log { "Capturer" };
    FramerateCounter loss_image_framerate {};

    std::unique_ptr<Cap> capturer;
    Cap::Config hikcamera_config;

    std::chrono::seconds reconnect_wait_seconds { 1 };

    util::spsc_queue<Image*, 10> capture_queue;
    std::jthread runtime_thread;

    auto initialize(const Config& config) noexcept -> std::expected<void, std::string> {
        using namespace std::chrono_literals;

        loss_image_framerate.enable = config.print_loss_framerate;
        loss_image_framerate.set_intetval(
            std::chrono::seconds { config.print_loss_framerate_interval_seconds });

        // Not connect here, to quick launch
        capturer = std::make_unique<Cap>();
        config.transform_to(hikcamera_config);

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
        auto capture_failed_limit = TimesLimit { 3 };

        auto failed_callback = [&](const std::string& msg) {
            if (capture_failed_limit.tick() == false) {
                log.error("Failed to capture image {} times", capture_failed_limit.count);
                log.error("- Newest error: {}", msg);
                log.error("- Reconnect capturer now...");

                std::ignore = capturer->deinitialize();
                rclcpp::sleep_for(reconnect_wait_seconds);
                capture_failed_limit.reset();
            }
        };

        // Reconnect context
        auto error_limit = TimesLimit { 3 };

        auto reconnect = [&] {
            if (!capturer->initialized()) {
                auto ret = capturer->deinitialize();
            }
            if (auto result = capturer->initialize(hikcamera_config)) {
                log.info("Connect to capturer successfully");
                error_limit.reset();
                error_limit.enable();
            } else {
                if (error_limit.tick()) {
                    log.error("Failed to reconnect to capturer, retry soon");
                    log.error("- Error: {}", result.error());
                } else if (error_limit.enabled()) {
                    error_limit.disable();
                    log.error("{} times, stop printing errors", error_limit.count);
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

auto Capturer::initialize(const Config& config) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(config);
}

auto Capturer::fetch_image() noexcept -> ImageUnique { return pimpl->fetch_image(); }

Capturer::Capturer() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Capturer::~Capturer() noexcept = default;

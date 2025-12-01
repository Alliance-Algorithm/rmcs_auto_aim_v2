#include "capturer.hpp"
#include "module/capturer/common.hpp"
#include "module/capturer/hikcamera.hpp"
#include "module/capturer/local_video.hpp"
#include "module/debug/framerate.hpp"
#include "utility/logging/printer.hpp"
#include "utility/singleton/running.hpp"
#include "utility/thread/spsc_queue.hpp"
#include "utility/times_limit.hpp"

#include <rclcpp/utilities.hpp>
#include <thread>

using namespace rmcs::kernel;
using namespace rmcs::cap;

struct Capturer::Impl {
    using RawImage = Image*;

    std::unique_ptr<Interface> interface;

    Printer log { "Capturer" };
    FramerateCounter loss_image_framerate {};

    std::chrono::milliseconds reconnect_wait_interval { 500 };

    util::spsc_queue<Image*, 10> capture_queue;
    std::jthread runtime_thread;

    auto initialize(const YAML::Node& yaml) noexcept -> Result try {
        auto source = yaml["source"].as<std::string>();

        auto instantitation_result = std::expected<void, std::string> {
            std::unexpected { "Unknown capturer source or not implemented source" },
        };

        // > 「系统实例化！」
        // > 不觉得这个名字很帅么
        auto system_instantiation = [&, this]<class Impl>(const std::string& source) {
            using Instance = cap::Adapter<Impl>;

            auto instance = std::make_unique<Instance>();
            auto result   = instance->configure_yaml(yaml[source]);
            if (!result.has_value()) {
                instantitation_result = std::unexpected { result.error() };
                return;
            }
            instantitation_result = {};

            interface = std::move(instance);
        };

        /*  */ if (source == "hikcamera") {
            system_instantiation.operator()<Hikcamera>(source);
        } else if (source == "local_video") {
            system_instantiation.operator()<LocalVideo>(source);
        } else if (source == "images") {
        }

        if (!instantitation_result.has_value()) {
            return std::unexpected { instantitation_result.error() };
        }

        auto show_loss_framerate          = yaml["show_loss_framerate"].as<bool>();
        auto show_loss_framerate_interval = yaml["show_loss_framerate_interval"].as<int>();

        loss_image_framerate.enable = show_loss_framerate;
        loss_image_framerate.set_intetval(
            std::chrono::milliseconds { show_loss_framerate_interval });

        reconnect_wait_interval =
            std::chrono::milliseconds { yaml["reconnect_wait_interval"].as<int>() };

        runtime_thread = std::jthread {
            [this](const auto& t) { runtime_task(t); },
        };
        return {};

    } catch (const std::exception& e) {
        return std::unexpected { e.what() };
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
        auto capture_failed_limit = util::TimesLimit { 3 };

        auto failed_callback = [&](const std::string& msg) {
            if (capture_failed_limit.tick() == false) {
                log.error("Failed to capture image {} times", capture_failed_limit.count);
                log.error("- Newest error: {}", msg);
                log.error("- Reconnect capturer now...");

                rclcpp::sleep_for(reconnect_wait_interval);
                capture_failed_limit.reset();
            }
        };

        // Reconnect context
        auto error_limit = util::TimesLimit { 3 };

        auto reconnect = [&] {
            if (auto result = interface->connect()) {
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
            rclcpp::sleep_for(reconnect_wait_interval);
        };

        for (;;) {
            if (!util::get_running()) [[unlikely]]
                break;

            if (token.stop_requested()) [[unlikely]]
                break;

            if (!interface->connected()) {
                reconnect();
                continue;
            }

            if (auto result = interface->wait_image()) {
                success_callback(std::move(*result));
            } else {
                failed_callback(result.error());
            }
        }
        log.info("Because the cancellation operation [capturer thread] has ended");
    }
};

auto Capturer::initialize(const Yaml& config) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(config);
}

auto Capturer::fetch_image() noexcept -> ImageUnique { return pimpl->fetch_image(); }

Capturer::Capturer() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Capturer::~Capturer() noexcept = default;

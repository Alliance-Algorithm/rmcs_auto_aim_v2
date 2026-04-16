#include "capturer.hpp"
#include "kernel/feishu.hpp"
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
    std::chrono::milliseconds trigger_sync_max_age { 50 };

    util::spsc_queue<Image*, 10> capture_queue;
    std::jthread runtime_thread;
    Channel<util::CameraTriggerEvent> camera_trigger_channel;
    bool enable_trigger_sync { false };
    std::uint64_t last_bound_trigger_seq_ { 0 };

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

        auto trigger_sync_config = yaml["enable_trigger_sync"].as<bool>();
        enable_trigger_sync      = (source == "hikcamera" && trigger_sync_config);

        auto show_loss_framerate          = yaml["show_loss_framerate"].as<bool>();
        auto show_loss_framerate_interval = yaml["show_loss_framerate_interval"].as<int>();

        loss_image_framerate.enable = show_loss_framerate;
        loss_image_framerate.set_interval(
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

    auto runtime_task(const std::stop_token& token) noexcept -> void {
        log.info("[Capturer runtime thread] starts");

        // Success context
        auto missing_trigger_limit  = util::TimesLimit { 3 };
        auto bind_trigger_timestamp = [&](std::unique_ptr<Image>& image) {
            if (!enable_trigger_sync) {
                return;
            }

            auto capture_timestamp = image->get_timestamp();
            if (auto trigger = camera_trigger_channel.fetch_latest_matching(
                    [&](const util::CameraTriggerEvent& candidate) {
                        return candidate.seq > last_bound_trigger_seq_
                            && candidate.timestamp <= capture_timestamp
                            && capture_timestamp - candidate.timestamp <= trigger_sync_max_age;
                    })) {
                image->set_timestamp(trigger->timestamp);
                last_bound_trigger_seq_ = trigger->seq;
                missing_trigger_limit.reset();
                missing_trigger_limit.enable();
                return;
            }

            if (missing_trigger_limit.tick()) {
                log.warn("No camera trigger event is available for the captured image");
            } else if (missing_trigger_limit.enabled()) {
                missing_trigger_limit.disable();
                log.warn(
                    "{} times, stop printing trigger-sync warnings", missing_trigger_limit.count);
            }
        };

        auto success_callback = [&](std::unique_ptr<Image> image) {
            bind_trigger_timestamp(image);
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
                interface->disconnect();

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

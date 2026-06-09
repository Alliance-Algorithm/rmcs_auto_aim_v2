#include "capturer.hpp"
#include "module/capturer/common.hpp"
#include "module/capturer/hikcamera.hpp"
#include "module/capturer/local_video.hpp"
#include "utility/framerate.hpp"
#include "utility/image/image.details.hpp"
#include "utility/image/recorder.hpp"
#include "utility/logging/printer.hpp"
#include "utility/service.hpp"
#include "utility/singleton/running.hpp"
#include "utility/thread/spsc_queue.hpp"
#include "utility/times_limit.hpp"

#include <ranges>
#include <thread>
#include <vector>

using namespace rmcs::kernel;
using namespace rmcs::cap;

struct Capturer::Impl {
    using RawImage = Image*;

    struct Config : util::Serializable {
        std::string source;
        bool show_loss_framerate;
        int show_loss_framerate_interval;
        int reconnect_wait_interval;
        bool record_enable;
        std::vector<std::string> saving_pathes;
        int max_duration_seconds;
        std::size_t record_fps;
        std::uintmax_t max_videos_size_gb;

        // clang-format off
        static constexpr std::tuple metas {
            &Config::source,                    "source",
            &Config::show_loss_framerate,       "show_loss_framerate",
            &Config::show_loss_framerate_interval, "show_loss_framerate_interval",
            &Config::reconnect_wait_interval,   "reconnect_wait_interval",
            &Config::record_enable,             "record_enable",
            &Config::saving_pathes,             "saving_pathes",
            &Config::max_duration_seconds,      "max_duration_seconds",
            &Config::record_fps,                "record_fps",
            &Config::max_videos_size_gb,        "max_videos_size_gb",
        };
        // clang-format on
    } config;

    std::unique_ptr<Interface> interface;

    Printer log { "Capturer" };
    FramerateCounter loss_image_framerate { };

    util::spsc_queue<Image*, 10> capture_queue;
    std::jthread runtime_thread;

    VideoRecorder recorder { };

    std::atomic<std::int8_t> record_request = 0;
    std::jthread service_thread { [this](const std::stop_token& token) {
        using namespace util;
        auto service = Service {
            Named<"cap"> { },
            Action { Named<"record">(),
                [this](std::string_view data) {
                    constexpr auto kT = std::array { "1", "true" };
                    constexpr auto kF = std::array { "0", "false" };

                    const auto lower = std::ranges::to<std::string>(data
                        | std::views::drop_while([](char c) { return c == '\n' || c == '\r'; })
                        | std::views::reverse
                        | std::views::drop_while([](char c) { return c == '\n' || c == '\r'; })
                        | std::views::reverse | std::views::transform(::tolower));
                    const auto equal = [&](std::string_view target) {
                        return std::ranges::equal(lower, target);
                    };
                    /*^^*/ if (std::ranges::any_of(kT, equal)) {
                        record_request = +1;
                    } else if (std::ranges::any_of(kF, equal)) {
                        record_request = -1;
                    } else {
                        log.error("无法处理请求: {}, 忽略", data);
                    }
                } },
        };
        while (!token.stop_requested()) {
            service.spin();

            using namespace std::chrono_literals;
            std::this_thread::sleep_for(100ms);
        }
    } };

    auto initialize(const YAML::Node& yaml) noexcept -> Result try {
        if (auto result = config.serialize(yaml); !result.has_value()) {
            return std::unexpected { result.error() };
        }

        auto& source = config.source;

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
            instantitation_result = { };

            interface = std::move(instance);
        };

        /*^^*/ if (source == "hikcamera") {
            system_instantiation.operator()<Hikcamera>(source);
        } else if (source == "local_video") {
            system_instantiation.operator()<LocalVideo>(source);
        } else if (source == "images") {
        }

        if (!instantitation_result.has_value()) {
            return std::unexpected { instantitation_result.error() };
        }

        loss_image_framerate.enable = config.show_loss_framerate;
        loss_image_framerate.set_interval(
            std::chrono::milliseconds { config.show_loss_framerate_interval });

        runtime_thread = std::jthread {
            [this](const auto& t) { runtime_task(t); },
        };

        recorder.update_config({
            .directories     = config.saving_pathes,
            .max_duration    = std::chrono::seconds { config.max_duration_seconds },
            .record_fps      = config.record_fps,
            .max_videos_size = config.max_videos_size_gb * 1024 * 1024 * 1024,
        });

        return { };

    } catch (const std::exception& e) {
        return std::unexpected { e.what() };
    }

    ~Impl() noexcept {
        runtime_thread.request_stop();
        if (runtime_thread.joinable()) {
            runtime_thread.join();
        }
    }

    // 为了实时性，一般取最新的帧
    auto fetch_image() noexcept -> ImageUnique {
        auto result = ImageUnique { nullptr };
        auto image  = RawImage { };
        while (capture_queue.pop(image)) {
            result = ImageUnique { image };
        }
        return result;
    }

    auto runtime_task(const std::stop_token& token) noexcept -> void {
        log.info("[Capturer runtime thread] starts");

        // Success context
        auto success_callback = [&](std::unique_ptr<Image> image) {
            auto newest = image.release();
            recorder.tick(newest->details().mat);

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

                std::this_thread::sleep_for(
                    std::chrono::milliseconds { config.reconnect_wait_interval });
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
            std::this_thread::sleep_for(
                std::chrono::milliseconds { config.reconnect_wait_interval });
        };

        while (util::get_running() && !token.stop_requested()) {
            if (!interface->connected()) {
                reconnect();
                continue;
            }

            if (config.record_enable) {
                const auto request = record_request.load(std::memory_order::relaxed);
                /*^^*/ if (request == +1) {
                    // 开始录制
                    if (!recorder.recording()) {
                        auto result = recorder.start();
                        if (!result.has_value()) {
                            log.error("无法开启录制，报错如下: {}", result.error());
                        } else {
                            log.info("成功开启录制");
                        }
                    }
                } else if (request == -1) {
                    // 停止录制
                    recorder.stop();
                    log.info("{}", recorder.status());
                }
                record_request.store(0, std::memory_order::relaxed);
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

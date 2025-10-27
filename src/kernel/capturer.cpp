#include "capturer.hpp"
#include "modules/capturer/hikcamera.hpp"
#include "utility/image.impl.hpp"

#include <boost/lockfree/spsc_queue.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

#include <thread>

using namespace rmcs::kernel;
using boost::lockfree::capacity;
using boost::lockfree::spsc_queue;

struct Capturer::Impl {
    spsc_queue<Image*, capacity<10>> buffer;

    std::jthread working_thread;

    std::unique_ptr<capturer::Camera> capturer = std::make_unique<capturer::Camera>();
    capturer::Config config;

    rclcpp::Logger logger = rclcpp::get_logger("capturer");

    ~Impl() noexcept { stop_working(); }

    auto initialize() -> std::expected<void, std::string> {

        using namespace std::chrono_literals;
        config.exposure_us  = 2000.;
        config.invert_image = false;
        config.gain         = 16.9807;
        config.framerate    = 80;

        if (auto ret = capturer->initialize(config); !ret) {
            return std::unexpected { ret.error() };
        }
        return {};
    }

    auto fetch() noexcept {
        Image* result = nullptr;
        buffer.pop(result);
        return std::unique_ptr<Image> { result };
    }

    auto blocking_reconnect_capturer(std::stop_token const& token = {}) const noexcept -> void {

        if (capturer->initialized())
            if (auto ret = capturer->deinitialize(); !ret) {
                RCLCPP_WARN(logger, "Failed to deinitialize: %s", ret.error().data());
            }

        using namespace std::chrono_literals;
        auto waiting_rate = rclcpp::WallRate { 1s };

        const auto living = bool { rclcpp::ok() && !token.stop_requested() };
        while (!capturer->initialized() && living) {
            if (auto result = capturer->initialize(config)) {
                RCLCPP_INFO(logger, "Successfully re-established connection");
                RCLCPP_INFO(logger, "\n%s", result.value().c_str());
            } else {
                RCLCPP_ERROR(logger, "Failed to connect: %s", result.error().data());
                waiting_rate.sleep();
            }
        }
    }

    std::size_t failed_count = 0;
    auto working_task(std::stop_token const& token) noexcept {
        while (rclcpp::ok() && !token.stop_requested()) {

            auto mat = capturer->read_image();
            if (!mat.has_value()) {

                auto error = mat.error();
                RCLCPP_ERROR(logger, "Failed to capture image: %s", error.data());

                if (failed_count++ > 5) {
                    failed_count = 0;
                    blocking_reconnect_capturer(token);
                }

                using namespace std::chrono_literals;
                std::this_thread::sleep_for(1s);

                continue;
            }

            auto* image { new Image };
            image->details().mat = *mat;
            if (!buffer.push(image)) {
                delete image;
                RCLCPP_WARN(logger, "Failed to push image, the buffer is full");
            }
        }
    }

    auto start_working() noexcept -> void {
        working_thread = std::jthread {
            [this](std::stop_token const& token) { working_task(token); },
        };
    }
    auto stop_working() noexcept -> void {
        working_thread.request_stop();
        if (working_thread.joinable()) {
            working_thread.join();
        }
    }
};

auto Capturer::initialize() noexcept -> std::expected<void, std::string> {
    return pimpl->initialize();
}
auto Capturer::start_working() noexcept -> void {
    pimpl->start_working(); //
}
auto Capturer::stop_working() noexcept -> void {
    pimpl->stop_working(); //
}
auto Capturer::fetch() noexcept -> std::unique_ptr<Image> {
    return pimpl->fetch(); //
}

Capturer::Capturer() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Capturer::~Capturer() noexcept = default;

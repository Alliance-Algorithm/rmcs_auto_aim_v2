#include "capturer.hpp"
#include "modules/capturer/hikcamera.hpp"
#include "utility/image.impl.hpp"

#include <boost/lockfree/spsc_queue.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <yaml-cpp/node/node.h>

#include <thread>

using namespace rmcs::kernel;
using boost::lockfree::capacity;
using boost::lockfree::spsc_queue;

struct Capturer::Impl {
    spsc_queue<Image*, capacity<10>> buffer;

    std::jthread working_thread;

    std::unique_ptr<capturer::HikcameraCap> capturer = //
        std::make_unique<capturer::HikcameraCap>();

    capturer::HikcameraCap::Profile capturer_profile;
    std::chrono::milliseconds capturer_timeout;
    std::float_t capturer_framerate = 80;

    rclcpp::Logger logger = rclcpp::get_logger("capturer");

    ~Impl() noexcept { stop_working(); }

    auto initialize() -> std::expected<void, std::string> {

        // TODO:

        using namespace std::chrono_literals;
        capturer_profile.exposure_time = 2ms;
        capturer_profile.invert_image  = false;
        capturer_profile.gain          = 16.9807;

        capturer_framerate = 80;
        capturer_timeout   = 1s;

        blocking_reconnect_capturer();
        return {};
    }

    auto fetch() noexcept {
        Image* result = nullptr;
        buffer.pop(result);
        return std::unique_ptr<Image> { result };
    }

    auto blocking_reconnect_capturer(std::stop_token const& token = {}) const noexcept -> void {
        if (capturer->initialized()) {
            capturer->reset();
        }

        using namespace std::chrono_literals;
        auto waiting_rate = rclcpp::WallRate { 3s };

        while (!capturer->initialized() && rclcpp::ok() && !token.stop_requested()) {
            if (auto result = capturer->initialize(capturer_profile)) {
                RCLCPP_INFO(logger, "Successfully re-established connection");
                capturer->set_frame_rate_inner_trigger_mode(capturer_framerate);
            } else {
                RCLCPP_ERROR(logger, "Cap: %s", result.error().data());
                waiting_rate.sleep();
            }
        }
    }

    auto working_task(std::stop_token const& token) noexcept {
        while (rclcpp::ok() && !token.stop_requested()) {

            auto result = capturer->read(capturer_timeout);
            if (!result.has_value()) {
                auto error = result.error();
                RCLCPP_ERROR(logger, "Failed to capture image: %s", error.data());

                blocking_reconnect_capturer(token);
                continue;
            }

            auto* image { new Image };
            image->details().mat = std::move(result.value());
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

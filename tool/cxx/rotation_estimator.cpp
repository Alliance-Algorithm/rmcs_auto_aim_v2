#include "module/perception/rotation_estimator.hpp"

#include <hikcamera/capturer.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <print>
#include <thread>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

using RotationMessage = geometry_msgs::msg::Vector3Stamped;

struct LatestFrame {
    std::mutex mutex;
    std::condition_variable cv;

    std::unique_ptr<rmcs::Image> image;
    std::uint64_t sequence { 0 };
};

struct Statistics {
    std::atomic<std::size_t> captured { 0 };
    std::atomic<std::size_t> dropped { 0 };
    std::atomic<double> read_time_sum_ms { 0.0 };

    std::size_t processed { 0 };
    std::size_t estimated { 0 };
    double update_time_sum_ms { 0.0 };

    int tracked_points { 0 };
    int inlier_points { 0 };
    double confidence { 0.0 };

    auto add_read_time(std::chrono::steady_clock::duration duration) -> void {
        read_time_sum_ms.fetch_add(std::chrono::duration<double, std::milli>(duration).count(),
            std::memory_order::relaxed);
    }

    auto add_update_time(std::chrono::steady_clock::duration duration) -> void {
        update_time_sum_ms += std::chrono::duration<double, std::milli>(duration).count();
    }

    auto record_estimate(const rmcs::RotationEstimator::Addition& addition) -> void {
        estimated += 1;
        tracked_points = addition.tracked_points;
        inlier_points  = addition.inlier_points;
        confidence     = addition.confidence;
    }

    auto record_failed_estimate(const rmcs::RotationEstimator::Addition& addition) -> void {
        tracked_points = addition.tracked_points;
        inlier_points  = addition.inlier_points;
        confidence     = 0.0;
    }

    auto print_and_reset() -> void {
        const auto captured_count = captured.exchange(0, std::memory_order::relaxed);
        const auto dropped_count  = dropped.exchange(0, std::memory_order::relaxed);
        const auto read_time_sum  = read_time_sum_ms.exchange(0.0, std::memory_order::relaxed);

        const auto read_divisor   = captured_count == 0 ? 1.0 : static_cast<double>(captured_count);
        const auto update_divisor = processed == 0 ? 1.0 : static_cast<double>(processed);

        std::println("[rotation_estimator] captured={}, processed={}, estimates={}, dropped={}, "
                     "tracked={}, "
                     "inliers={}, confidence={:.2f}, read={:.2f}ms, update={:.2f}ms",
            captured_count, processed, estimated, dropped_count, tracked_points, inlier_points,
            confidence, read_time_sum / read_divisor, update_time_sum_ms / update_divisor);

        processed          = 0;
        estimated          = 0;
        update_time_sum_ms = 0.0;
    }
};

constexpr auto kCameraMatrix = std::array<double, 9> {
    1.722231837421459e+03,
    0.0,
    7.013056440882832e+02,
    0.0,
    1.724876404292754e+03,
    5.645821718351237e+02,
    0.0,
    0.0,
    1.0,
};

constexpr auto kDistortCoeff = std::array<double, 5> {
    -0.064232403853946,
    -0.087667493884102,
    0.0,
    0.0,
    0.792381808294582,
};

} // namespace

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    auto node      = std::make_shared<rclcpp::Node>("rotation_estimator_tool");
    auto publisher = node->create_publisher<RotationMessage>(
        "/rmcs/auto_aim/visual_rotation_rate", rclcpp::QoS { 10 }.best_effort().keep_last(10));
    auto clock = rclcpp::Clock { RCL_STEADY_TIME };

    auto config = hikcamera::Config {
        .timeout_ms      = 2'000,
        .exposure_us     = 1'500,
        .framerate       = 120,
        .fixed_framerate = true,
    };
    config.framerate = 165;

    auto camera = hikcamera::Camera { };
    camera.configure(config);
    if (auto result = camera.connect(); !result) {
        std::println("[rotation_estimator] Failed to connect camera: {}", result.error());
        rclcpp::shutdown();
        return 1;
    }

    auto estimator = rmcs::RotationEstimator { };
    estimator.configure(kCameraMatrix);
    estimator.configure(kDistortCoeff);

    auto latest_frame = LatestFrame { };
    auto statistics   = Statistics { };

    auto capture_thread = std::jthread { [&](const std::stop_token& stop) {
        while (!stop.stop_requested() && rclcpp::ok()) {
            const auto read_begin = std::chrono::steady_clock::now();
            auto captured         = camera.read_image_with_timestamp();
            const auto read_end   = std::chrono::steady_clock::now();
            statistics.add_read_time(read_end - read_begin);

            if (!captured) {
                std::println("[rotation_estimator] Failed to read image: {}", captured.error());
                continue;
            }

            auto image       = std::make_unique<rmcs::Image>();
            image->mat       = captured->mat.clone();
            image->timestamp = captured->timestamp;

            {
                auto lock = std::lock_guard { latest_frame.mutex };
                if (latest_frame.image) statistics.dropped.fetch_add(1, std::memory_order::relaxed);
                latest_frame.image = std::move(image);
                latest_frame.sequence += 1;
            }
            statistics.captured.fetch_add(1, std::memory_order::relaxed);
            latest_frame.cv.notify_one();
        }
    } };

    auto last_print    = std::chrono::steady_clock::now();
    auto last_sequence = std::uint64_t { 0 };

    std::println("[rotation_estimator] Publishing /rmcs/auto_aim/visual_rotation_rate");
    std::println("[rotation_estimator] Press Ctrl+C to quit");

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        auto image = std::unique_ptr<rmcs::Image> { };
        {
            auto lock = std::unique_lock { latest_frame.mutex };
            latest_frame.cv.wait_for(lock, std::chrono::milliseconds { 5 },
                [&] { return latest_frame.sequence != last_sequence || !rclcpp::ok(); });
            if (latest_frame.sequence == last_sequence || !latest_frame.image) continue;

            last_sequence = latest_frame.sequence;
            image         = std::move(latest_frame.image);
        }

        statistics.processed += 1;

        const auto update_begin = std::chrono::steady_clock::now();
        auto estimate           = estimator.update(*image);
        const auto update_end   = std::chrono::steady_clock::now();
        statistics.add_update_time(update_end - update_begin);
        const auto& addition = estimator.addition();

        if (estimate) {
            statistics.record_estimate(addition);

            auto message            = RotationMessage { };
            message.header.stamp    = clock.now();
            message.header.frame_id = "camera_link";
            message.vector.x        = estimate->yaw_rate;
            message.vector.y        = estimate->pitch_rate;
            message.vector.z        = estimate->roll_rate;
            publisher->publish(message);
        } else {
            statistics.record_failed_estimate(addition);
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_print >= std::chrono::seconds { 1 }) {
            statistics.print_and_reset();
            last_print = now;
        }
    }

    capture_thread.request_stop();
    latest_frame.cv.notify_one();
    if (capture_thread.joinable()) capture_thread.join();
    std::ignore = camera.disconnect();
    rclcpp::shutdown();
    std::println("[rotation_estimator] Stopped");
    return 0;
}

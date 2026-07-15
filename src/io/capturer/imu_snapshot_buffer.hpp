#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <new>
#include <optional>

#include <eigen3/Eigen/Geometry>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/board_clock.hpp>
#include <rmcs_msgs/imu_snapshot.hpp>
#include <rmcs_utility/ring_buffer.hpp>

namespace rmcs {

class ImuSnapshotBuffer {
public:
    using TimePoint = rmcs_msgs::BoardClock::time_point;

    explicit ImuSnapshotBuffer(rmcs_executor::Component& component, std::size_t size)
        : buffer_ { size } {
        component.register_input("/gimbal/auto_aim/imu_snapshot", imu_snapshot_input_);
    }

    auto pop(TimePoint time) -> std::optional<Eigen::Quaterniond> {
        const auto guard = std::scoped_lock { consumer_mutex_ };

        auto view = buffer_.const_readable_view();
        if (view.empty()) return std::nullopt;

        const auto right = std::lower_bound(
            view.begin(), view.end(), time,
            [](const OrientationSnapshot& snapshot, TimePoint target_time) {
                return snapshot.timestamp < target_time;
            });

        if (right == view.end()) {
            const auto latest = right - 1;
            buffer_.pop_front_until(latest);
            return std::nullopt;
        }

        if (right->timestamp == time) {
            auto orientation = right->orientation;
            buffer_.pop_front_until(right);
            return orientation;
        }

        if (right == view.begin()) return std::nullopt;

        const auto left     = right - 1;
        const auto duration = std::chrono::duration<double>(right->timestamp - left->timestamp);
        const auto elapsed  = std::chrono::duration<double>(time - left->timestamp);
        auto orientation = left->orientation.slerp(elapsed.count() / duration.count(), right->orientation);
        buffer_.pop_front_until(left);
        return orientation;
    }

    auto pop_latest() -> std::optional<Eigen::Quaterniond> {
        const auto guard = std::scoped_lock { consumer_mutex_ };

        auto view = buffer_.const_readable_view();
        if (view.empty()) return std::nullopt;

        const auto latest = view.end() - 1;
        auto orientation    = latest->orientation;
        buffer_.pop_front_until(latest);
        return orientation;
    }

private:
    void push(const rmcs_msgs::ImuSnapshot& snapshot) {
        if (snapshot.timestamp < last_push_time) return;

        while (!buffer_.emplace_back_n(
            [&snapshot](std::byte* storage) noexcept {
                new (storage) OrientationSnapshot { snapshot.orientation, snapshot.timestamp };
            },
            1)) {
            const auto guard = std::scoped_lock { consumer_mutex_ };
            if (buffer_.writable()) continue;
            buffer_.pop_front_n([](OrientationSnapshot&&) noexcept { }, buffer_.max_size() >> 1);
        }

        last_push_time = snapshot.timestamp;
    }

    rmcs_executor::Component::EventInputInterface<rmcs_msgs::ImuSnapshot> imu_snapshot_input_ {
        [this](const rmcs_msgs::ImuSnapshot& snapshot) { push(snapshot); } };
    TimePoint last_push_time = TimePoint::min();

    struct OrientationSnapshot {
        Eigen::Quaterniond orientation;
        rmcs_msgs::BoardClock::time_point timestamp;
    };
    rmcs_utility::RingBuffer<OrientationSnapshot> buffer_;
    std::mutex consumer_mutex_;
};

} // namespace rmcs

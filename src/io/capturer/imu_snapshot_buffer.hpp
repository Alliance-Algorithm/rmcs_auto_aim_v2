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

    struct Snapshot {
        Eigen::Quaterniond orientation;
        Eigen::Vector3d gyro_body;
    };

    explicit ImuSnapshotBuffer(rmcs_executor::Component& component, std::size_t size)
        : buffer_ { size } {
        component.register_input("/gimbal/auto_aim/imu_snapshot", imu_snapshot_input_);
    }

    auto pop(TimePoint time) -> std::optional<Snapshot> {
        const auto guard = std::scoped_lock { consumer_mutex_ };

        auto view = buffer_.const_readable_view();
        if (view.empty()) return std::nullopt;

        const auto right = std::lower_bound(view.begin(), view.end(), time,
            [](const OrientationSnapshot& snapshot, TimePoint target_time) {
                return snapshot.timestamp < target_time;
            });

        if (right == view.end()) {
            const auto latest = right - 1;
            buffer_.pop_front_until(latest);
            return std::nullopt;
        }

        if (right->timestamp == time) {
            auto snapshot = Snapshot { right->orientation, right->gyro_body };
            buffer_.pop_front_until(right);
            return snapshot;
        }

        if (right == view.begin()) return std::nullopt;

        const auto left     = right - 1;
        const auto duration = std::chrono::duration<double>(right->timestamp - left->timestamp);
        const auto elapsed  = std::chrono::duration<double>(time - left->timestamp);
        const auto ratio    = elapsed.count() / duration.count();
        auto snapshot       = Snapshot {
            left->orientation.slerp(ratio, right->orientation),
            left->gyro_body + (right->gyro_body - left->gyro_body) * ratio,
        };
        buffer_.pop_front_until(left);
        return snapshot;
    }

    /// 按 host 时钟索引弹出快照，用于无法获得 BoardClock 对齐关系的 fallback 场景。
    /// 目标时刻新于最新快照时，钳制到最新快照；早于最老快照时返回 nullopt。
    auto pop_host(std::chrono::steady_clock::time_point time) -> std::optional<Snapshot> {
        const auto guard = std::scoped_lock { consumer_mutex_ };

        auto view = buffer_.const_readable_view();
        if (view.empty()) return std::nullopt;

        const auto right = std::lower_bound(view.begin(), view.end(), time,
            [](const OrientationSnapshot& snapshot,
                std::chrono::steady_clock::time_point target_time) {
                return snapshot.host_timestamp < target_time;
            });

        if (right == view.end()) {
            const auto latest = right - 1;
            auto snapshot     = Snapshot { latest->orientation, latest->gyro_body };
            buffer_.pop_front_until(latest);
            return snapshot;
        }

        if (right->host_timestamp == time) {
            auto snapshot = Snapshot { right->orientation, right->gyro_body };
            buffer_.pop_front_until(right);
            return snapshot;
        }

        if (right == view.begin()) return std::nullopt;

        const auto left = right - 1;
        const auto duration =
            std::chrono::duration<double>(right->host_timestamp - left->host_timestamp);
        const auto elapsed = std::chrono::duration<double>(time - left->host_timestamp);
        const auto ratio   = elapsed.count() / duration.count();
        auto snapshot      = Snapshot {
            left->orientation.slerp(ratio, right->orientation),
            left->gyro_body + (right->gyro_body - left->gyro_body) * ratio,
        };
        buffer_.pop_front_until(left);
        return snapshot;
    }

private:
    void push(const rmcs_msgs::ImuSnapshot& snapshot) {
        if (snapshot.timestamp < last_push_time) return;

        const auto host_timestamp = std::chrono::steady_clock::now();
        while (!buffer_.emplace_back_n(
            [&snapshot, host_timestamp](std::byte* storage) noexcept {
                new (storage) OrientationSnapshot { snapshot.orientation, snapshot.gyro_body,
                    snapshot.timestamp, host_timestamp };
            },
            1)) {
            const auto guard = std::scoped_lock { consumer_mutex_ };
            if (buffer_.writable()) continue;
            buffer_.pop_front_n([](OrientationSnapshot&&) noexcept { }, buffer_.max_size() >> 1);
        }

        last_push_time = snapshot.timestamp;
    }

    rmcs_executor::Component::EventInputInterface<rmcs_msgs::ImuSnapshot> imu_snapshot_input_ {
        [this](const rmcs_msgs::ImuSnapshot& snapshot) { push(snapshot); }
    };
    TimePoint last_push_time = TimePoint::min();

    struct OrientationSnapshot {
        Eigen::Quaterniond orientation;
        Eigen::Vector3d gyro_body;
        rmcs_msgs::BoardClock::time_point timestamp;
        std::chrono::steady_clock::time_point host_timestamp;
    };
    rmcs_utility::RingBuffer<OrientationSnapshot> buffer_;
    std::mutex consumer_mutex_;
};

} // namespace rmcs

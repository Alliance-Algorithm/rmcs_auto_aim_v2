#pragma once
#include "utility/clock.hpp"
#include "utility/shared/interprocess.hpp"
#include <chrono>
#include <deque>
#include <optional>
#include <ranges>

namespace rmcs::kernel {

template <class T>
concept timestamp_trait = requires(const T& data) {
    { data.timestamp } -> std::convertible_to<util::Clock::time_point>;
};

template <class T>
concept context_trait = requires {
    { T::kLabel } -> std::convertible_to<const char*>;
    { T::kLength } -> std::convertible_to<std::size_t>;
};

template <context_trait SendT, context_trait RecvT>
class Feishu {
private:
    using Timestamp = util::Clock::time_point;
    using Duration  = Timestamp::duration;

    using SendClient = shm::Client<SendT>::Send;
    SendClient send_client { };

    using RecvClient = shm::Client<RecvT>::Recv;
    RecvClient recv_client { };

    mutable Timestamp latest_timestamp = Timestamp::min();
    std::deque<RecvT> recv_buffer { };

public:
    template <std::invocable<const RecvT&> F>
    auto recv(F&& f) const noexcept {
        auto with = [this, f = std::forward<F>(f)](const RecvT& data) {
            latest_timestamp = data.timestamp;
            f(data);
        };
        recv_client.with_read(with);
    }

    template <std::invocable<SendT&> F>
    auto send(F&& f) noexcept {
        send_client.with_write(std::forward<F>(f));
    }
    auto send(const SendT& data) noexcept {
        Feishu::send([&](SendT& buffer) { buffer = data; });
    }

    auto start() noexcept -> bool {
        auto send_opened = send_client.opened() || send_client.open(SendT::kLabel);
        auto recv_opened = recv_client.opened() || recv_client.open(RecvT::kLabel);
        return send_opened && recv_opened;
    }

    auto updated() const noexcept { return recv_client.is_updated(); }

    /// @return bool 是否收到数据
    auto heartbeat() noexcept -> bool {
        if (start() && updated()) {
            recv([this](const RecvT& data) {
                if (recv_buffer.size() >= RecvT::kLength) {
                    recv_buffer.pop_front();
                }
                recv_buffer.push_back(data);
            });
            return true;
        }
        return false;
    }

    /// 搜索离传入时间戳最近的消息
    auto search(Timestamp target, Duration max = std::chrono::seconds { 5 }) const
        -> std::optional<RecvT> {
        static_assert(timestamp_trait<RecvT>);

        if (target > latest_timestamp) return std::nullopt;

        auto to_return = std::optional<RecvT> { };
        auto shortest  = max;
        for (const RecvT& data : recv_buffer | std::views::reverse) {
            const auto timestamp = Timestamp { data.timestamp };

            // 区间范围外的，跳过
            if (timestamp > target + max) continue;
            if (timestamp < target - max) break;

            // 更新最近值
            const auto interval = timestamp - target;
            if (shortest > std::chrono::abs(interval)) {
                shortest = std::chrono::abs(interval);

                to_return = data;
            }

            // 间隔为负值时，已经过 target
            if (interval < Duration::zero()) break;
        }
        return to_return;
    }
};

}

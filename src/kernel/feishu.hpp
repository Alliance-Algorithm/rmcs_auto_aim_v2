#pragma once

#include "utility/shared/context.hpp"
#include "utility/shared/interprocess.hpp"
#include <optional>
#include <type_traits>
#include <utility>

namespace rmcs::kernel {

template <typename T>
constexpr const char* shm_name = nullptr;

inline constexpr std::size_t kControlStateHistoryCapacity  = 4096;
inline constexpr std::size_t kCameraTriggerHistoryCapacity = 512;

template <>
constexpr auto shm_name<util::AutoAimState> = "/shm_autoaim_state";

template <>
constexpr auto shm_name<util::ControlState> = "/shm_control_state";

template <>
constexpr auto shm_name<util::CameraTriggerEvent> = "/shm_camera_trigger";

enum class RuntimeRole { AutoAim, Control };

template <typename T>
struct ChannelTraits {
    using SendClient = typename rmcs::shm::Client<T>::Send;
    using RecvClient = typename rmcs::shm::Client<T>::Recv;
};

template <>
struct ChannelTraits<util::ControlState> {
    using SendClient =
        typename rmcs::shm::HistoryClient<util::ControlState, kControlStateHistoryCapacity>::Send;
    using RecvClient =
        typename rmcs::shm::HistoryClient<util::ControlState, kControlStateHistoryCapacity>::Recv;
};

template <>
struct ChannelTraits<util::CameraTriggerEvent> {
    using SendClient = typename rmcs::shm::HistoryClient<util::CameraTriggerEvent,
        kCameraTriggerHistoryCapacity>::Send;
    using RecvClient = typename rmcs::shm::HistoryClient<util::CameraTriggerEvent,
        kCameraTriggerHistoryCapacity>::Recv;
};

template <typename T>
class Channel {
public:
    static_assert(shm_name<T> != nullptr, "Channel<T> requires shm_name<T> specialization");

    using SendClient = typename ChannelTraits<T>::SendClient;
    using RecvClient = typename ChannelTraits<T>::RecvClient;

    auto commit(const T& data) noexcept -> bool {
        if (!ensure_open(send_client_, shm_name<T>)) [[unlikely]]
            return false;

        if constexpr (requires { send_client_.push(data); }) {
            return send_client_.push(data);
        } else {
            send_client_.with_write([&](T& shared) { shared = data; });
            return true;
        }
    }

    auto fetch() noexcept -> const T& {
        if (!ensure_open(recv_client_, shm_name<T>)) return recv_buffer_;

        if constexpr (requires { recv_client_.latest(recv_buffer_); }) {
            recv_client_.latest(recv_buffer_);
        } else {
            recv_client_.with_read([&](const T& shared) { recv_buffer_ = shared; });
        }
        return recv_buffer_;
    }

    auto updated() noexcept -> bool {
        return ensure_open(recv_client_, shm_name<T>) && recv_client_.is_updated();
    }

    template <typename Predicate>
    auto fetch_latest_matching(Predicate&& predicate) noexcept -> std::optional<T> {
        if (!ensure_open(recv_client_, shm_name<T>)) {
            return std::nullopt;
        }

        auto buffer = T {};
        if constexpr (requires {
                          recv_client_.find_latest(std::forward<Predicate>(predicate), buffer);
                      }) {
            if (!recv_client_.find_latest(std::forward<Predicate>(predicate), buffer)) {
                return std::nullopt;
            }
        } else {
            return std::nullopt;
        }

        recv_buffer_ = buffer;
        return buffer;
    }

private:
    template <typename Client>
    static auto ensure_open(Client& client, const char* name) noexcept -> bool {
        return client.opened() || (name && client.open(name));
    }

    SendClient send_client_ {};
    RecvClient recv_client_ {};
    T recv_buffer_ {};
};

template <RuntimeRole Role>
class Feishu {
public:
    static_assert(Role == RuntimeRole::AutoAim || Role == RuntimeRole::Control,
        "Feishu<Role> only supports AutoAim and Control");

    using AutoAimState = util::AutoAimState;
    using ControlState = util::ControlState;

    using SendData = std::conditional_t<Role == RuntimeRole::AutoAim, AutoAimState, ControlState>;
    using RecvData = std::conditional_t<Role == RuntimeRole::AutoAim, ControlState, AutoAimState>;

    auto commit(SendData const& data) noexcept -> bool { return send_channel_.commit(data); }

    auto fetch() noexcept -> const RecvData& { return recv_channel_.fetch(); }

    auto updated() noexcept -> bool { return recv_channel_.updated(); }

    template <RuntimeRole R = Role>
    auto fetch_latest_before(util::Clock::time_point timestamp) noexcept
        -> std::enable_if_t<R == RuntimeRole::AutoAim, std::optional<ControlState>> {
        return recv_channel_.fetch_latest_matching(
            [&](const ControlState& state) { return state.timestamp <= timestamp; });
    }

private:
    Channel<SendData> send_channel_ {};
    Channel<RecvData> recv_channel_ {};
};

} // namespace rmcs::kernel

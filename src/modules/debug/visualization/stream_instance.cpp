#include "stream_instance.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>

using namespace rmcs::debug;

struct StreamSession::Impl final {
public:
    auto session_description_protocol() const noexcept -> std::expected<std::string, std::string> {

        if (!context || !context->opened()) {
            return std::unexpected { "Stream context is not initialized" };
        }

        const auto local_ipv4 = get_same_subnet_ipv4(context->stream_target().host);
        if (!local_ipv4) {
            return std::unexpected { std::format("Local IPv4 not found: {}", local_ipv4.error()) };
        }

        const auto result = context->session_description_protocol(*local_ipv4);
        if (!result) {
            return std::unexpected { std::string { result.error() } };
        }

        return *result;
    }

    auto initialize(StreamType type, const StreamTarget& target, const VideoFormat& format) noexcept
        -> void {
        context = std::make_unique<StreamContext>(type, format, target);
    }

private:
    struct NetworkInfo {
        in_addr_t address;
        in_addr_t netmask;
    };

    static auto get_network_info() -> std::expected<std::vector<NetworkInfo>, std::error_code> {
        ifaddrs* if_list;
        if (getifaddrs(&if_list) == -1) {
            return std::unexpected(std::error_code(errno, std::generic_category()));
        }

        std::vector<NetworkInfo> network_info_list;
        for (ifaddrs* if_ptr = if_list; if_ptr; if_ptr = if_ptr->ifa_next) {
            if (if_ptr->ifa_addr && if_ptr->ifa_addr->sa_family == AF_INET) {
                const auto* sa_in = reinterpret_cast<sockaddr_in*>(if_ptr->ifa_addr);
                const auto* sn_in = reinterpret_cast<sockaddr_in*>(if_ptr->ifa_netmask);
                network_info_list.push_back({ sa_in->sin_addr.s_addr, sn_in->sin_addr.s_addr });
            }
        }
        freeifaddrs(if_list);
        return network_info_list;
    }
    static auto get_same_subnet_ipv4(std::string_view target_ip_str)
        -> std::expected<std::string, std::string> {

        in_addr target_addr;
        if (inet_pton(AF_INET, target_ip_str.data(), &target_addr) != 1) {
            return std::unexpected(std::format("Invalid target IP address: {}", target_ip_str));
        }

        const auto network_info_expected = get_network_info();
        if (!network_info_expected) {
            return std::unexpected(std::format(
                "Failed to get network info: {}", network_info_expected.error().message()));
        }

        const auto& network_info_list = network_info_expected.value();
        const auto& target_addr_bin   = target_addr.s_addr;

        for (const auto& info : network_info_list) {
            if ((info.address & info.netmask) == (target_addr_bin & info.netmask)) {
                char local_ip[INET_ADDRSTRLEN];
                if (inet_ntop(AF_INET, &info.address, local_ip, sizeof(local_ip))) {
                    return std::string(local_ip);
                }
            }
        }

        return std::unexpected(
            std::format("Could not find a local IP in the same subnet as {}", target_ip_str));
    }

private:
    std::unique_ptr<StreamContext> context;
};

StreamSession::StreamSession(
    StreamType type, const StreamTarget& target, const VideoFormat& format) noexcept
    : pimpl { std::make_unique<Impl>() } {
    pimpl->initialize(type, target, format);
}

StreamSession::~StreamSession() noexcept = default;

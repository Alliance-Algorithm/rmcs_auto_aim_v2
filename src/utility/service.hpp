#pragma once

#include "utility/string.hpp"

#include <cctype>
#include <cerrno>
#include <charconv>
#include <filesystem>
#include <format>
#include <fstream>
#include <map>
#include <print>
#include <stdexcept>

#include <fcntl.h>
#include <poll.h>
#include <sys/stat.h>
#include <unistd.h>

namespace rmcs::util {

template <StaticString Name, typename Callback>
    requires std::invocable<Callback, std::string_view>
class Action {
public:
    static constexpr auto kNameView = std::string_view { Named<Name>::kView };

    template <typename U>
        requires std::constructible_from<std::decay_t<Callback>, U&&>
    explicit Action(Named<Name>, U&& callback)
        : callback_ { std::forward<U>(callback) } { }

    ~Action() { close(); }

    void open(std::string_view prefix) {
        auto path = std::format("{}{}", prefix, kNameView);

        if (::mkfifo(path.c_str(), 0666)) {
            if (errno != EEXIST) {
                throw std::runtime_error { std::format("管道无法打开: {}", path) };
            }
        }

        fd_ = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
    }

    void close() {
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
    }

    void spin_once() {
        if (fd_ < 0) return;

        auto pfd = pollfd { fd_, POLLIN, 0 };
        if (::poll(&pfd, 1, 0) > 0) {
            auto n = ::read(fd_, buf_, sizeof(buf_));
            if (n > 0) {
                callback_(std::string_view { buf_, static_cast<std::size_t>(n) });
            }
        }
    }

private:
    std::decay_t<Callback> callback_;
    int fd_ = -1;
    char buf_[256];
};

template <StaticString Name, typename U>
Action(Named<Name>, U&&) -> Action<Name, std::decay_t<U>>;

namespace details {
    template <typename T>
    struct is_action_impl : std::false_type { };

    template <StaticString Name, typename Callback>
    struct is_action_impl<Action<Name, Callback>> : std::true_type { };

    template <typename T>
    concept is_action = is_action_impl<std::remove_cvref_t<T>>::value;

}

template <StaticString Name, typename Callback>
    requires std::invocable<Callback&, std::string_view>
auto make_action(Callback&& callback) {
    return Action { Named<Name> { }, std::forward<Callback>(callback) };
}

template <StaticString Name, typename Callback>
    requires std::invocable<Callback&> && (!std::invocable<Callback&, std::string_view>)
auto make_action(Callback&& callback) {
    return Action { Named<Name> { },
        [callback = std::forward<Callback>(callback)](std::string_view) mutable { callback(); } };
}

template <StaticString Name, typename T, typename Callback>
    requires std::same_as<T, bool> && std::invocable<Callback&, bool>
auto make_action(Callback&& callback) {
    return Action { Named<Name> { },
        [callback = std::forward<Callback>(callback)](std::string_view data) mutable {
            data = trim(data);

            if (data == "1") callback(true); // NOLINT(bugprone-branch-clone)
            else if (data == "0") callback(false); // NOLINT(bugprone-branch-clone)
            else if (ascii_iequals(data, "true") || ascii_iequals(data, "on")) callback(true);
            else if (ascii_iequals(data, "false") || ascii_iequals(data, "off")) {
                callback(false);
            }
        } };
}

template <StaticString Name, typename T, typename Callback>
    requires std::same_as<T, int> && std::invocable<Callback&, int>
auto make_action(Callback&& callback) {
    return Action { Named<Name> { },
        [callback = std::forward<Callback>(callback)](std::string_view data) mutable {
            data = trim(data);

            int value         = 0;
            const auto result = std::from_chars(data.data(), data.data() + data.size(), value);
            if (result.ec == std::errc { } && result.ptr == data.data() + data.size()) {
                callback(value);
            }
        } };
}

template <StaticString Name, typename T, typename Callback>
    requires std::same_as<T, double> && std::invocable<Callback&, double>
auto make_action(Callback&& callback) {
    return Action { Named<Name> { },
        [callback = std::forward<Callback>(callback)](std::string_view data) mutable {
            data = trim(data);

            double value      = 0;
            const auto result = std::from_chars(data.data(), data.data() + data.size(), value);
            if (result.ec == std::errc { } && result.ptr == data.data() + data.size()) {
                callback(value);
            }
        } };
}

template <StaticString Name, details::is_action... Actions>
class Service {
public:
    static constexpr auto kNameView = std::string_view { Named<Name>::kView };

    explicit Service(Named<Name>, Actions&&... context)
        : actions_ { std::forward<Actions>(context)... } {
        namespace fs = std::filesystem;

        service_location_ = std::format("/tmp/autoaim/{}/", kNameView);

        fs::remove_all(service_location_);
        fs::create_directories(service_location_);

        {
            auto actions_path     = std::format("{}actions", service_location_);
            auto actions_ofstream = std::ofstream { actions_path };
            if (!actions_ofstream) throw std::runtime_error { "无法创建 actions 文件" };

            std::apply(
                [&](auto&... actions) {
                    (actions.open(service_location_), ...);
                    (std::println(actions_ofstream, "- {}", actions.kNameView), ...);
                },
                actions_);
            actions_ofstream.close();
        }

        {
            auto context_path = std::format("{}context", service_location_);
            context_fd_       = ::open(context_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
            if (context_fd_ < 0) throw std::runtime_error { "无法创建 context 文件" };
        }
    }

    ~Service() {
        std::apply([](auto&... actions) { (actions.close(), ...); }, actions_);
        if (context_fd_ >= 0) ::close(context_fd_);

        std::filesystem::remove_all(service_location_);
    }

    void spin_once() {
        std::apply([](auto&... actions) { (actions.spin_once(), ...); }, actions_);

        if (context_updated_) {
            context_updated_ = false;

            std::string buffer;
            for (const auto& [name, content] : context_map_)
                std::format_to(std::back_inserter(buffer), "{}: {}\n", name, content);

            if (::ftruncate(context_fd_, 0) != 0 || ::lseek(context_fd_, 0, SEEK_SET) != 0) return;
            if (::write(context_fd_, buffer.data(), buffer.size()) < 0) return;
        }
    }

    void update_context(const std::string& key, const std::string& value) {
        context_map_[key] = value;
        context_updated_  = true;
    }

private:
    std::string service_location_;
    std::tuple<std::decay_t<Actions>...> actions_;

    int context_fd_ = -1;
    std::map<std::string, std::string> context_map_;
    bool context_updated_ = false;
};

template <StaticString Name, details::is_action... Actions>
auto make_service(Actions&&... actions) {
    return Service { Named<Name> { }, std::forward<Actions>(actions)... };
}

}

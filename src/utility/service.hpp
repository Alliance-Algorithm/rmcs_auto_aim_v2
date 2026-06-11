#pragma once
#include "utility/duck_type.hpp"
#include "utility/string.hpp"

#include <cerrno>
#include <filesystem>
#include <format>
#include <fstream>
#include <print>
#include <stdexcept>
#include <unordered_map>

#include <fcntl.h>
#include <poll.h>
#include <sys/stat.h>
#include <unistd.h>

namespace rmcs::util {

namespace details {
    struct action_checker {
        template <typename T>
        struct result {
            static constexpr bool v = requires(T t) { t.spin_once(); };
        };
    };
}

template <StaticString Name, typename Callback>
    requires std::invocable<Callback, std::string_view>
struct Action {
    static constexpr auto kNameView = std::string_view { Named<Name>::kView };

    Callback callback;
    int fd = -1;
    char buf[256];

    explicit Action(Named<Name>, Callback&& callback)
        : callback { std::forward<Callback>(callback) } { }

    ~Action() {
        if (fd >= 0) ::close(fd);
    }

    auto open(std::string_view prefix) -> void {
        auto path = std::format("{}{}", prefix, kNameView);

        if (::mkfifo(path.c_str(), 0666)) {
            if (errno != EEXIST) {
                throw std::runtime_error { std::format("管道无法打开: {}", path) };
            }
        }

        fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
    }

    auto spin_once() -> void {
        if (fd < 0) return;

        auto pfd = pollfd { fd, POLLIN, 0 };
        if (::poll(&pfd, 1, 0) > 0) {
            auto n = ::read(fd, buf, sizeof(buf));
            if (n > 0) {
                callback(std::string_view { buf, static_cast<std::size_t>(n) });
            }
        }
    }
};

template <StaticString Name, typename... Actions>
class Service {
    static constexpr auto kNameView = std::string_view { Named<Name>::kView };

private:
    /// @NOTE: 这里的析构顺序，必须保证 actions > on_exit > location
    std::string service_location { };
    struct OnExit {
        const std::string& to_remove;
        ~OnExit() {
            namespace fs = std::filesystem;
            if (fs::exists(to_remove)) {
                fs::remove_all(to_remove);
            }
        }
    } on_exit { service_location };

    duck_array<Actions...> actions;

    bool context_updated = false;
    std::string context_path;
    std::unordered_map<std::string, std::string> context_map;

    template <typename Action>
    auto write_action_path(std::ofstream& file, std::string_view prefix) -> void {
        file << "- " << prefix << Action::kNameView << "\n";
    }

public:
    explicit Service(Named<Name>, Actions&&... context)
        : actions { details::action_checker { }, std::forward<Actions>(context)... } {
        namespace fs = std::filesystem;

        service_location = std::format("/tmp/autoaim/{}/", kNameView);
        if (fs::exists(service_location)) {
            fs::remove_all(service_location);
        }

        fs::create_directories(service_location);

        // 打开所有 action 的 FIFO
        actions.foreach ([&](auto& action) { action.open(service_location); });

        // 创建 actions 列表文件（只读）
        auto actions_path = std::format("{}actions", service_location);
        auto filestream   = std::ofstream { actions_path };
        if (!filestream) {
            throw std::runtime_error { std::format("无法创建 actions 文件") };
        }
        (write_action_path<Actions>(filestream, service_location), ...);
        filestream.close();

        fs::permissions(actions_path,
            fs::perms::owner_read | fs::perms::group_read | fs::perms::others_read,
            fs::perm_options::replace);

        context_path = std::format("{}context", service_location);
    }

    auto spin() -> void {
        actions.foreach ([](auto& action) { action.spin_once(); });

        namespace fs = std::filesystem;
        if (context_updated) {
            context_updated = false;

            auto stream = std::ofstream { context_path, std::ios::out | std::ios::trunc };
            for (const auto& [name, content] : context_map) {
                std::println(stream, "{}: {}", name, content);
            }
            stream.flush();
        }
    }

    auto update_later(const std::string& name, const std::string& content) {
        context_map[name] = content;
        context_updated   = true;
    }
};

}

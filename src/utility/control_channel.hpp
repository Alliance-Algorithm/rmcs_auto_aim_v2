#pragma once

#include <algorithm>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace rmcs::util {

[[nodiscard]] inline auto unix_ms() -> std::int64_t {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch())
        .count();
}

[[nodiscard]] inline auto json_string(std::string_view text) -> std::string {
    auto result = std::string { "\"" };
    for (const unsigned char ch : text) {
        if (ch == '"' || ch == '\\') {
            result += '\\';
            result += static_cast<char>(ch);
        } else if (ch < 0x20) {
            result += std::format("\\u{:04x}", ch);
        } else {
            result += static_cast<char>(ch);
        }
    }
    return result + '"';
}

// Control endpoint polled by tool/res/start-streamer: commands arrive as lines on the `commands`
// FIFO, and `state.json` is replaced atomically together with a pid/updated_at heartbeat.
class ControlChannel {
public:
    ControlChannel(const std::filesystem::path& root, std::string_view name)
        : directory_ { root / name } {
        if (!valid_name(name)) {
            throw std::runtime_error { std::format("Invalid control channel name: {}", name) };
        }

        try {
            std::filesystem::create_directories(directory_);
            lock_fd_ = ::open((directory_ / ".lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0644);
            if (lock_fd_ < 0 || ::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
                throw std::runtime_error { std::format(
                    "Cannot lock {}; already running?", directory_.string()) };
            }
            owns_lock_ = true;

            const auto commands_path = directory_ / "commands";
            struct stat info { };
            if (::lstat(commands_path.c_str(), &info) == 0) {
                if (!S_ISFIFO(info.st_mode)) {
                    throw std::runtime_error { std::format(
                        "{} exists and is not a FIFO", commands_path.string()) };
                }
                std::filesystem::remove(commands_path);
            }
            if (::mkfifo(commands_path.c_str(), 0660) != 0) {
                throw std::runtime_error { std::format(
                    "Cannot create FIFO {}", commands_path.string()) };
            }
            owns_fifo_ = true;
            fifo_fd_   = ::open(commands_path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
            if (fifo_fd_ < 0) {
                throw std::runtime_error { std::format(
                    "Cannot open FIFO {}", commands_path.string()) };
            }
        } catch (...) {
            close();
            throw;
        }
    }

    ~ControlChannel() { close(); }

    ControlChannel(const ControlChannel&)                    = delete;
    auto operator=(const ControlChannel&) -> ControlChannel& = delete;

    [[nodiscard]] auto directory() const noexcept -> const std::filesystem::path& {
        return directory_;
    }

    // `fields` are the JSON object members that follow the heartbeat.
    void write_state(std::string_view fields) const {
        const auto json = std::format(R"({{"pid":{},"updated_at":{},{}}})"
                                      "\n",
            ::getpid(), static_cast<double>(unix_ms()) / 1000.0, fields);

        const auto temporary_path = directory_ / "state.json.tmp";
        {
            auto output = std::ofstream { temporary_path, std::ios::trunc };
            output << json;
            output.close();
            if (!output) {
                throw std::runtime_error { std::format(
                    "Cannot write {}", temporary_path.string()) };
            }
        }
        std::filesystem::rename(temporary_path, directory_ / "state.json");
    }

    // Lines longer than kMaxLineLength are dropped as a whole.
    template <std::invocable<const std::string&> Handler>
    void drain_commands(Handler&& handle_line) {
        char buffer[512];
        // Bound the work per call even if a writer keeps the pipe continuously busy.
        for (int reads = 0; reads < 8; ++reads) {
            const auto count = ::read(fifo_fd_, buffer, sizeof(buffer));
            if (count <= 0) break;
            for (const auto ch : std::string_view { buffer, static_cast<std::size_t>(count) }) {
                if (ch == '\n') {
                    if (!discard_line_ && !pending_line_.empty()) handle_line(pending_line_);
                    pending_line_.clear();
                    discard_line_ = false;
                } else if (!discard_line_) {
                    if (pending_line_.size() >= kMaxLineLength) {
                        pending_line_.clear();
                        discard_line_ = true;
                    } else {
                        pending_line_ += ch;
                    }
                }
            }
        }
    }

    void wait_for_command(std::chrono::milliseconds timeout) const {
        auto descriptor = pollfd { fifo_fd_, POLLIN, 0 };
        (void)::poll(&descriptor, 1, static_cast<int>(timeout.count()));
    }

    [[nodiscard]] static auto valid_request_id(std::string_view id) -> bool {
        return id.size() <= 64 && std::ranges::all_of(id, [](char ch) {
            return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9')
                || ch == '-';
        });
    }

private:
    static constexpr std::size_t kMaxLineLength = 256;

    [[nodiscard]] static auto valid_name(std::string_view name) -> bool {
        const auto valid_start = [](char ch) {
            return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || ch == '_';
        };
        return !name.empty() && valid_start(name.front())
            && std::ranges::all_of(
                name, [&](char ch) { return valid_start(ch) || (ch >= '0' && ch <= '9'); });
    }

    void close() noexcept {
        if (fifo_fd_ >= 0) ::close(fifo_fd_);
        if (owns_lock_) {
            auto ignored = std::error_code { };
            if (owns_fifo_) std::filesystem::remove(directory_ / "commands", ignored);
            std::filesystem::remove(directory_ / "state.json", ignored);
            std::filesystem::remove(directory_ / "state.json.tmp", ignored);
        }
        // Keep the lock file so a new owner cannot race an unlinked lock inode.
        if (lock_fd_ >= 0) ::close(lock_fd_);
        fifo_fd_   = -1;
        lock_fd_   = -1;
        owns_lock_ = false;
        owns_fifo_ = false;
    }

    const std::filesystem::path directory_;
    int lock_fd_       = -1;
    int fifo_fd_       = -1;
    bool owns_lock_    = false;
    bool owns_fifo_    = false;
    bool discard_line_ = false;
    std::string pending_line_;
};

} // namespace rmcs::util

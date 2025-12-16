#pragma once
#include <cstdlib>
#include <filesystem>
#include <string_view>

class AssetsManager {
public:
    explicit AssetsManager(
        std::string_view env = "TEST_ASSETS_ROOT", std::filesystem::path fallback = "/tmp/auto_aim")
        : root_ { discover_root(env, std::move(fallback)) } { }

    const std::filesystem::path& root() const noexcept { return root_; }
    std::filesystem::path path(std::string_view filename) const { return root_ / filename; }

private:
static std::filesystem::path discover_root(
    std::string_view env, std::filesystem::path fallback) {
    if (const char* v = std::getenv(std::string(env).c_str()); v && *v) return std::filesystem::path { v };
    return fallback;
}

    std::filesystem::path root_;
};

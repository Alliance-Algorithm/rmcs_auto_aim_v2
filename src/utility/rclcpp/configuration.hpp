#pragma once
#include "utility/logging/printer.hpp"
#include "utility/rclcpp/parameters.hpp"

#include <cstdlib>
#include <filesystem>

#include <yaml-cpp/yaml.h>

namespace rmcs::util {

inline auto configs() -> YAML::Node {
    auto printer = Printer { "configs" };

    const auto location = std::filesystem::path {
        Parameters::share_location(),
    };

    { // [1] 尝试读取环境变量
        if (const auto env = std::getenv("AUTOAIM_CONFIG"); env && *env) {
            const auto env_path = std::filesystem::path { env };
            const auto target   = env_path.is_absolute() ? env_path : location / env_path;
            if (std::filesystem::exists(target)) {
                printer.info("AUTOAIM_CONFIG={} 已作为配置文件加载", target.string());
                return YAML::LoadFile(target);
            }
            const auto error =
                std::format("AUTOAIM_CONFIG={} (解析为 {}) 不存在", env, target.string());
            throw std::runtime_error { error };
        }
    }
    { // [2] 按照优先级读取配置文件
        const auto candidates = std::array {
            location / "custom.yaml",
            location / "custom.yml",
            location / "config.override.yaml",
            location / "config.override.yml",

            location / "config.yaml",
            location / "config.yml",
        };
        for (const auto& config : candidates) {
            if (std::filesystem::exists(config)) {
                printer.info("{} 已作为配置文件加载", config.string());
                return YAML::LoadFile(config);
            }
        }
    }
    throw std::runtime_error { "未能加载配置文件，请检查 config/ 下是否存在配置" };
}

}

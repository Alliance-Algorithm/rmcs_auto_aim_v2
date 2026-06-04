#pragma once
#include "utility/logging/printer.hpp"
#include "utility/rclcpp/parameters.hpp"

#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace rmcs::util {

inline auto configs() -> YAML::Node {
    auto printer = Printer { "configs" };

    const auto location = std::filesystem::path {
        Parameters::share_location(),
    };
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
            printer.info("{} is loaded as config", config.string());
            return YAML::LoadFile(config);
        }
    }
    throw std::runtime_error { "未能加载配置文件，请检查 config/ 下是否存在配置" };
}

}

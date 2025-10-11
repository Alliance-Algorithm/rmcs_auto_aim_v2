#pragma once
#include <expected>
#include <yaml-cpp/yaml.h>

namespace rmcs {

struct ThreadAllocation {
    std::size_t identifier  = 2;
    std::size_t transformer = 1;
    std::size_t tracker     = 1;

    static auto serialized_from(const YAML::Node& node) noexcept
        -> std::expected<ThreadAllocation, std::string_view>;
};

}

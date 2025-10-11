#include "threads.hpp"

using namespace rmcs;

auto ThreadAllocation::serialized_from(const YAML::Node& node) noexcept
    -> std::expected<ThreadAllocation, std::string_view> {
    if (!node.IsMap()) {
        return std::unexpected("YAML node is not a map");
    }
    try {
        ThreadAllocation result;
        result.identifier  = node["identifier"].as<std::size_t>();
        result.transformer = node["transformer"].as<std::size_t>();
        result.tracker     = node["tracker"].as<std::size_t>();
        return result;
    } catch (const YAML::Exception& e) {
        return std::unexpected("Failed to parse ThreadAllocation");
    }
}

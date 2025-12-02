#pragma once
#include <expected>
#include <yaml-cpp/yaml.h>

namespace rmcs::util {

enum class SerializeTfError {
    UNMATCHED_LINKS_IN_YAML,
    UNMATCHED_LINKS_IN_TREE,
};

// @note:
// 错误不打断序列化，全部序列化完后再返回结果
// 一些错误是可以被接受的
template <class Root>
auto serialize_from(const YAML::Node& yaml) noexcept -> std::expected<void, SerializeTfError> {
    // ...
}

template <class Root>
auto serialize_from(Root, const YAML::Node& yaml) noexcept {
    return serialize_from<Root>(yaml);
}

}

#pragma once
#include "utility/string.hpp"
#include "utility/yaml/eigen.hpp"
#include <concepts>
#include <eigen3/Eigen/Geometry>
#include <expected>
#include <map>
#include <string>
#include <string_view>
#include <type_traits>
#include <yaml-cpp/yaml.h>

namespace rmcs::util {

enum class SerializeTfError {
    UNMATCHED_LINKS_IN_YAML, // YAML 中有未匹配的 transform
    UNMATCHED_LINKS_IN_TREE, // Tree 中有节点在 YAML 中找不到对应 transform
    INVALID_YAML_FORMAT,     // YAML 格式无效（不是序列格式）
    MISSING_REQUIRED_FIELDS, // Transform 缺少必需的字段（parent 或 child）
    TYPE_MISMATCH,           // Transform 数据类型与节点 State 类型不匹配
};
constexpr auto to_string(SerializeTfError e) {
    switch (e) {
    case SerializeTfError::UNMATCHED_LINKS_IN_YAML:
        return "Yaml 中存在未匹配上的变换";
    case SerializeTfError::UNMATCHED_LINKS_IN_TREE:
        return "Tree 中存在未匹配上的变换";
    case SerializeTfError::INVALID_YAML_FORMAT:
        return "错误的 Yaml 解析";
    case SerializeTfError::MISSING_REQUIRED_FIELDS:
        return "Yaml 中变换的格式不正确，需要 child 与 parent";
    case SerializeTfError::TYPE_MISMATCH:
        return "Yaml 中的参数类型与节点的 state 无法匹配";
    }
    return "无法抵达的彼岸";
}

template <class Root>
struct SerializeTfAdapter {
    template <StaticString name>
    static constexpr auto set_state(const auto& state) noexcept {
        using StateType = std::decay_t<decltype(state)>;
        static_assert(
            requires(const StateType& s) { Root::template set_state<name>(s); },
            "\nRoot 类型不支持 set_state 接口，它应该形如："
            "\n  Root::template set_state<StaticString>(const State&)\n");
        Root::template set_state<name>(state);
    }

    template <StaticString name, typename T>
    static constexpr auto get_state() noexcept {
        static_assert(
            requires {
                { Root::template get_state<name, T>() } -> std::convertible_to<T>;
            },
            "\nRoot 类型不支持 get_state 接口，它应该形如："
            "\n  Root::template get_state<StaticString, Type>() -> Type\n");
        return Root::template get_state<name, T>();
    }

    template <typename F>
    static constexpr auto foreach_df_with_parent(F&& f) noexcept -> void {
        static_assert(
            requires { Root::foreach_df_with_parent(std::declval<F>()); },
            "\nRoot 类型不支持 foreach_df_with_parent 接口，它应该形如："
            "\n  Root::foreach_df_with_parent(F&& f)"
            "\n其中 f 应该形如："
            "\n  []<class Joint>(std::string_view parent_name){ ... }\n");
        Root::foreach_df_with_parent(std::forward<F>(f));
    }
};

// @note:
// - 错误不打断序列化，全部序列化完后再返回结果
// - 一些错误是可以被接受的
// - yaml 应该是 transform 列表的根节点（序列），而不是包含 transforms 字段的对象
template <class Root>
auto serialize_from(const YAML::Node& yaml) noexcept -> std::expected<void, SerializeTfError> {
    using Adapter = SerializeTfAdapter<Root>;

    // 检查 YAML 格式：应该是序列
    if (!yaml.IsSequence()) {
        return std::unexpected { SerializeTfError::INVALID_YAML_FORMAT };
    }

    // 建立 parent -> child 的映射，存储 transform 数据
    auto transform_map      = std::map<std::string, std::map<std::string, YAML::Node>> {};
    auto has_missing_fields = bool { false };
    for (const auto& transform_node : yaml) {
        if (!transform_node["parent"] || !transform_node["child"]) {
            has_missing_fields = true;
            continue;
        }
        auto parent = std::string { transform_node["parent"].as<std::string>() };
        auto child  = std::string { transform_node["child"].as<std::string>() };

        auto node_copy               = YAML::Node { transform_node };
        transform_map[parent][child] = node_copy;
    }

    auto has_unmatched_yaml = bool { false };
    auto has_type_mismatch  = bool { false };

    // 通用的安全设置状态 lambda，统一处理 try-catch
    auto safe_set_state = [&]<typename Func>(Func&& func) noexcept -> void {
        try {
            std::forward<Func>(func)();
        } catch (...) {
            has_type_mismatch = true;
        }
    };

    // 使用 foreach_df_with_parent 遍历 tree 并设置状态
    Adapter::foreach_df_with_parent([&]<class T>(std::string_view parent_name) {
        auto parent_str = std::string { parent_name };
        auto child_str  = std::string { T::name };

        // 根节点跳过（没有 parent）
        if (parent_str.empty()) {
            return;
        }

        // 查找 YAML 中是否有对应的 transform
        if (auto parent_it = transform_map.find(parent_str); parent_it != transform_map.end()) {
            if (auto child_it = parent_it->second.find(child_str);
                child_it != parent_it->second.end()) {
                const auto& node = child_it->second;

                using State = typename T::State;
                if constexpr (std::is_same_v<State, Eigen::Isometry3d>) {
                    if (node["t"] && node["q"]) {
                        safe_set_state([&] {
                            auto t = read_eigen_translation<double>(node["t"]);
                            auto q = read_eigen_orientation<double>(node["q"]);
                            Adapter::template set_state<T::static_name>(
                                Eigen::Isometry3d { Eigen::Translation3d { t } * q });
                        });
                    } else if (node["q"]) {
                        safe_set_state([&] {
                            auto q = read_eigen_orientation<double>(node["q"]);
                            Adapter::template set_state<T::static_name>(Eigen::Isometry3d { q });
                        });
                    } else if (node["t"]) {
                        safe_set_state([&] {
                            auto t = read_eigen_translation<double>(node["t"]);
                            Adapter::template set_state<T::static_name>(
                                Eigen::Isometry3d { Eigen::Translation3d { t } });
                        });
                    }
                } else if constexpr (std::is_same_v<State, Eigen::Quaterniond>) {
                    if (node["q"]) {
                        safe_set_state([&] {
                            auto q = read_eigen_orientation<double>(node["q"]);
                            Adapter::template set_state<T::static_name>(q);
                        });
                    }
                } else if constexpr (std::is_same_v<State, Eigen::Translation3d>) {
                    if (node["t"]) {
                        safe_set_state([&] {
                            auto t = read_eigen_translation<double>(node["t"]);
                            Adapter::template set_state<T::static_name>(Eigen::Translation3d { t });
                        });
                    }
                } else if constexpr (std::is_same_v<State, Eigen::Vector3d>) {
                    if (node["t"]) {
                        safe_set_state([&] {
                            auto t = read_eigen_translation<double>(node["t"]);
                            Adapter::template set_state<T::static_name>(t);
                        });
                    }
                }

                // 标记已使用
                parent_it->second.erase(child_it);
                if (parent_it->second.empty()) {
                    transform_map.erase(parent_it);
                }
            }
        }
        // 注意：根节点（parent_name 为空）或找不到匹配的 transform 不报错
        // 因为 YAML 中可能不包含所有节点的 transform
    });

    // 检查是否有未使用的 YAML transforms
    if (!transform_map.empty()) {
        has_unmatched_yaml = true;
    }

    // 返回错误（优先级：格式错误 > 类型错误 > 缺失字段 > 未匹配）
    if (has_type_mismatch) {
        return std::unexpected { SerializeTfError::TYPE_MISMATCH };
    }
    if (has_missing_fields) {
        return std::unexpected { SerializeTfError::MISSING_REQUIRED_FIELDS };
    }
    if (has_unmatched_yaml) {
        return std::unexpected { SerializeTfError::UNMATCHED_LINKS_IN_YAML };
    }

    return {};
}

template <class Root>
auto serialize_from(Root, const YAML::Node& yaml) noexcept {
    return serialize_from<Root>(yaml);
}

}

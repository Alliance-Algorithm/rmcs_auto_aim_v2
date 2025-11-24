#pragma once

#include "utility/duck_type.hpp"
#include "utility/math/linear.hpp"
#include "utility/panic.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <ranges>
#include <string_view>
#include <utility>

namespace rmcs::util::tf::details {

constexpr auto make_hash(std::string_view tf_name) noexcept {
    std::uint64_t hash = 0xcbf29ce484222325ULL; // FNV offset basis
    for (char c : tf_name) {
        hash ^= static_cast<std::uint64_t>(static_cast<unsigned char>(c));
        hash *= 0x100000001b3ULL; // FNV prime
    }
    return hash;
}

template <typename T>
concept node_trait = requires(T t) {
    t.data.tf_name;
    t.childs;
};

struct node_checker_type {
    template <class T>
    struct result {
        constexpr static auto v = node_trait<T>;
    };
};
constexpr node_checker_type node_checker;

}
namespace rmcs::util {

struct JointTransform {
    Translation translation {};
    Orientation orientation {};

    auto inverse() const noexcept -> JointTransform;

    constexpr static auto Identity() { return JointTransform {}; }
};
auto operator*(const JointTransform&, const JointTransform&) -> JointTransform;

struct JointStatus {
    std::string_view tf_name;
};

template <tf::details::node_trait... node_tuple>
struct TfJoint {
    using name_type = std::string_view;

    static constexpr std::size_t kChildAmount = sizeof...(node_tuple);
    static constexpr std::size_t kTotalAmount = 1 + (0 + ... + node_tuple::kTotalAmount);

    JointStatus data;
    duck_array<node_tuple...> childs;

    constexpr explicit TfJoint(JointStatus data, node_tuple&&... childs) noexcept
        : data { data }
        , childs { tf::details::node_checker, std::forward<node_tuple>(childs)... } { }

    constexpr explicit TfJoint(name_type name, node_tuple&&... childs) noexcept
        : data { name }
        , childs { tf::details::node_checker, std::forward<node_tuple>(childs)... } { }

    constexpr auto has_duplicate_nodes() const noexcept {
        auto hash_table = std::array<std::uint64_t, kTotalAmount> {};
        auto hash_index = std::size_t { 0 };
        this->foreach ([&](const auto& node) { //
            hash_table[hash_index++] = tf::details::make_hash(node.data.tf_name);
        });
        hash_table[hash_index++] = tf::details::make_hash(data.tf_name);

        std::ranges::sort(hash_table);
        return std::ranges::adjacent_find(hash_table) != hash_table.end();
    }

    template <std::size_t n>
    constexpr auto at() const noexcept {
        static_assert(n < sizeof...(node_tuple), "Invalid index");
        return childs.template at<n>();
    }

    template <typename F>
    constexpr auto foreach (F&& func) const {
        childs.foreach ([&](const auto& joint) {
            func(joint);
            joint.foreach (func);
        });
    }

    constexpr auto find(name_type name, auto&& callback) const noexcept {
        if (name == data.tf_name) {
            callback(*this);
            return true;
        }

        auto found = bool { false };
        childs.foreach ([&](const auto& node) {
            if (found) return;
            if (node.data.tf_name == name) {
                callback(node);
                found = true;
            } else {
                found = node.find(name, callback);
            }
        });
        return found;
    }

    constexpr auto contains(name_type name) const noexcept -> bool {
        auto found = bool { false };
        find(name, [&](const auto&) { found = true; });
        return found;
    }

    constexpr std::size_t child_distance(name_type child) const noexcept {
        if (child == data.tf_name || kChildAmount == 0) return 0;

        auto result_distance = std::size_t { 0 };
        childs.foreach ([&](const auto& node) {
            if (result_distance != 0) return;

            if (node.data.tf_name == child) {
                result_distance = 1;
            } else if (auto sub = node.child_distance(child)) {
                result_distance = 1 + sub;
            }
        });
        return result_distance;
    }

    constexpr std::size_t child_distance(name_type parent, name_type child) const noexcept {
        auto result = std::size_t { 0 };
        find(parent, [&](const auto& node) {
            result = node.child_distance(child); //
        });
        return result;
    }

    constexpr auto find_nearest_common_ancestor(name_type a, name_type b) const noexcept
        -> std::optional<name_type> {
        auto path_a  = std::array<name_type, kTotalAmount> {};
        auto index_a = std::size_t { 0 };
        auto found_a = impl_find_path_up(a, path_a, index_a);

        auto path_b  = std::array<name_type, kTotalAmount> {};
        auto index_b = std::size_t { 0 };
        auto found_b = impl_find_path_up(b, path_b, index_b);

        if (!found_a || !found_b) return std::nullopt;

        const auto [reversed_a, length_reversed_a] = reverse_path(path_a);
        const auto [reversed_b, length_reversed_b] = reverse_path(path_b);

        auto common_ancestor = name_type {};
        for (auto [name_a, name_b] : std::views::zip(reversed_a, reversed_b)) {
            if (name_a == name_b) {
                common_ancestor = name_a;
            } else break;
        }
        return common_ancestor;
    }

    template <std::size_t length = kTotalAmount>
    constexpr auto find_child_path(name_type child_name) const noexcept
        -> std::optional<std::array<name_type, length>> {

        auto path = std::array<name_type, length> {};
        if (data.tf_name == child_name) return path;

        auto index = std::size_t { 0 };
        auto found = impl_find_path_up<length>(child_name, path, index);
        return found ? std::optional { path } : std::nullopt;
    }

    template <std::size_t length = kTotalAmount>
    constexpr auto find_child_path(name_type parent_name, name_type child_name) const noexcept
        -> std::optional<std::array<name_type, length>> {

        auto path = std::array<name_type, length> {};
        if (parent_name == child_name) return path;

        auto index = std::size_t { 0 };
        auto found = bool { false };
        find(parent_name, [&](const auto& node) {
            found = node.template impl_find_path_up<length>(child_name, path, index);
        });
        return found ? std::optional { path } : std::nullopt;
    }

    template <std::size_t length = kTotalAmount>
    constexpr auto find_path(name_type from, name_type to) const noexcept
        -> std::optional<std::array<name_type, length>> {

        auto path = std::array<name_type, length> {};
        if (from == to) [[unlikely]] {
            path[0] = from;
            return path;
        }

        auto ancestor = find_nearest_common_ancestor(from, to);
        if (ancestor->empty()) return std::nullopt;

        // from -> ancestor
        auto from_ancestor = find_child_path<length>(*ancestor, from);
        if (to == *ancestor) {
            std::ranges::copy(*from_ancestor, path.begin());
            return path;
        }

        // to -> ancestor
        auto to_ancestor = find_child_path<length>(*ancestor, to);

        auto remove_empty_joint = std::views::filter([&](auto name) { return !name.empty(); });
        auto remove_ancestor    = std::views::filter([&](auto name) { return name != *ancestor; });

        auto path_begin = *from_ancestor | remove_empty_joint | remove_ancestor;
        auto path_final = *to_ancestor | remove_empty_joint | std::views::reverse;

        auto it { path.begin() };
        it = std::ranges::copy(path_begin, it).out;
        it = std::ranges::copy(path_final, it).out;

        return path;
    }

public:
    template <std::size_t N = kTotalAmount>
    constexpr auto impl_find_path_up(name_type child, std::array<name_type, N>& result,
        std::size_t& index) const noexcept -> bool {
        if (data.tf_name == child) {
            if (index < N) result[index++] = data.tf_name;
            return true;
        }
        auto found = false;
        childs.foreach ([&](const auto& node) {
            if (found) return;
            if (node.impl_find_path_up(child, result, index)) {
                if (index < N) result[index++] = data.tf_name;
                found = true;
            }
        });
        return found;
    }

    template <std::size_t N = kTotalAmount>
    constexpr static auto reverse_path(const std::array<name_type, N>& path) noexcept {

        auto cleaned  = path | std::views::filter([](auto name) { return !name.empty(); });
        auto reversed = cleaned | std::views::reverse;

        auto result = std::array<name_type, N> {};
        auto length = std::ranges::distance(reversed);

        std::ranges::copy(reversed, result.begin());
        return std::tuple { result, length };
    }
};

template <tf::details::node_trait... node_tuple>
TfJoint(node_tuple...) -> TfJoint<std::decay_t<node_tuple>...>;

template <typename... node_tuple>
struct JointTransforms {
    using JointRoot = TfJoint<node_tuple...>;
    using name_type = std::string_view;

    constexpr static auto kTotalAmount = JointRoot::kTotalAmount;

    const JointRoot& joint_root;
    std::array<JointTransform, kTotalAmount> transforms;
    std::array<name_type, kTotalAmount> tf_names;

    consteval explicit JointTransforms(const JointRoot& root)
        : joint_root { root } {
        auto index        = std::size_t { 0 };
        tf_names[index++] = root.data.tf_name;
        root.foreach ([&](const auto& joint) { //
            tf_names[index++] = joint.data.tf_name;
        });
    }

    auto look_up_(name_type from, name_type to) const noexcept {
        if (from == to) return JointTransform::Identity();

        auto path = joint_root.find_path(from, to);
    }

    auto look_up(name_type from, name_type to) const noexcept -> JointTransform {
        if (from == to) {
            return JointTransform::Identity();
        }

        auto path_opt = joint_root.find_path(from, to);
        if (!path_opt.has_value()) {
            return JointTransform::Identity();
        }

        const auto& path  = path_opt.value();
        auto remove_empty = std::views::filter([](auto name) { return !name.empty(); });
        auto valid_path   = path | remove_empty;

        auto pairs = std::views::zip(valid_path, valid_path | std::views::drop(1));

        auto result = JointTransform::Identity();
        for (const auto& [current, next] : pairs) {
            auto distance = joint_root.child_distance(current, next);
            if (distance > 0) {
                result = result * (*this)[next].inverse();
            } else {
                result = result * (*this)[current];
            }
        }
        return result;
    }

    constexpr auto operator[](std::string_view tf_name) const -> const JointTransform& {
        return impl_find_transform(*this, tf_name);
    }
    constexpr auto operator[](std::string_view tf_name) -> JointTransform& {
        return impl_find_transform(*this, tf_name);
    }

    template <typename Self>
    constexpr static auto impl_find_transform(Self& self, std::string_view tf_name)
        -> decltype(auto) {
        for (std::size_t i = 0; i < self.tf_names.size(); ++i) {
            if (self.tf_names[i] == tf_name) {
                return self.transforms[i];
            }
        }
        util::panic("Unknown tf name");
    }
};
}

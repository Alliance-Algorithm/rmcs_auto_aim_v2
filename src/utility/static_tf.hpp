#pragma once
#include "utility/string.hpp"
#include <algorithm>
#include <tuple>

namespace rmcs::util::tf::details {

struct MonoState { };

template <typename T>
concept node_trait = requires {
    T::name;
    T::child_amount;
    T::total_amount;
};

struct node_checker_type {
    template <class T>
    struct result {
        constexpr static auto v = node_trait<T>;
    };
};
constexpr node_checker_type node_checker;

template <typename Token, StaticString name, typename T>
struct JointTransfroms {
    static inline T state = T {};
};

}
namespace rmcs::util {

template <StaticString Name, class T = tf::details::MonoState>
struct JointState { };

template <StaticString name_, class State_ = tf::details::MonoState, class... Ts_>
struct Joint {
    using State = State_;

    constexpr explicit Joint(JointState<name_, State_>, Ts_...) noexcept { }

    static constexpr std::size_t child_amount = sizeof...(Ts_);
    static constexpr std::size_t total_amount = 1 + (0 + ... + Ts_::total_amount);

    static constexpr auto name = name_.view();

    template <typename F>
    static constexpr auto foreach_df(F&& f) noexcept {
        f.template operator()<Joint>();

        auto recursion = [&]<class T>() { //
            T::foreach_df(std::forward<F>(f));
        };
        (recursion.template operator()<Ts_>(), ...);
    }

    template <StaticString query>
    static constexpr auto find(auto&& callback) noexcept -> bool {
        if constexpr (query == name) {
            callback.template operator()<Joint>();
            return true;
        }
        auto try_find = [&]<class T>() -> bool { //
            return T::template find<query>(callback);
        };
        return (false || ... || try_find.template operator()<Ts_>());
    }

    template <StaticString query>
    static constexpr auto contains() noexcept {
        if constexpr (query == name) return true;
        return (false || ... || Ts_::template contains<query>());
    }

    template <StaticString child>
    static constexpr auto child_distance() noexcept -> std::size_t {
        static_assert(Joint::contains<child>(), "Child was not found");
        return impl_traversal_child<child>();
    }

    template <StaticString parent, StaticString child>
    static constexpr auto child_distance() noexcept -> std::size_t {
        static_assert(Joint::contains<parent>(), "Parent was not found");
        static_assert(Joint::contains<child>(), "Child was not found");

        auto result = std::size_t { 0 };
        Joint::find<parent>([&]<class T>() {
            if (result == 0) result = T::template child_distance<child>();
        });
        return result;
    }

    template <StaticString child, std::size_t N = child_distance<child>()>
    static constexpr auto child_path(bool traversal_down = false) noexcept {
        static_assert(child != name, "Child should not be root");
        static_assert(Joint::contains<child>(), "Child was not found");

        auto result = std::array<std::string_view, N> {};

        auto index = traversal_down ? N - 1 : 0;
        auto step  = traversal_down ? -1 : +1;
        impl_traversal_child<child>([&]<class T>() {
            result[index] = T::name;
            index += step;
        });

        result[traversal_down ? N - 1 : 0] = child.view();
        return result;
    }

    template <StaticString parent, StaticString child>
    static constexpr auto child_path(bool traversal_down = false) noexcept {
        static_assert(parent != child, "They are the same node, bro");
        static_assert(Joint::contains<parent>(), "Parent was not found");
        static_assert(Joint::contains<child>(), "Child was not found");

        constexpr auto n { child_distance<parent, child>() };
        auto result = std::array<std::string_view, n> {};

        find<parent>([&]<class T>() {
            // Specify the result length to let lsp take a rest
            result = T::template child_path<child, n>(traversal_down);
        });
        return result;
    }

    template <StaticString a, StaticString b>
    static constexpr auto find_lca() noexcept {
        static_assert(a != b, "They are the same node");

        constexpr auto to_a = child_path<a>(true);
        constexpr auto to_b = child_path<b>(true);

        auto mismatch   = std::ranges::mismatch(to_a, to_b);
        auto common_len = std::ranges::distance(to_a.begin(), mismatch.in1);

        auto length = to_a.size() + to_b.size() - 2 * common_len;
        return std::make_tuple(*std::prev(mismatch.in1), length);
    }

    /// @note:
    /// callback(std::string_view, bool)
    /// - std::string_view name
    /// - bool need state inverse
    template <StaticString begin, StaticString final>
    static constexpr auto path(auto&& callback) noexcept
        requires requires { callback(std::string_view {}, bool {}); }
    {
        constexpr auto to_begin = child_path<begin>(true);
        constexpr auto to_final = child_path<final>(true);

        auto mismatch   = std::ranges::mismatch(to_begin, to_final);
        auto common_len = std::ranges::distance(to_begin.begin(), mismatch.in1);

        auto begin_to_lca = to_begin.size() - common_len;

        for (std::size_t i = 0; i < begin_to_lca; ++i) {
            auto side = to_begin.size() - 1;
            callback(to_begin[side - i], false);
        }
        for (std::size_t i = common_len; i < to_final.size(); ++i) {
            callback(to_final[i], true);
        }
    }

    template <StaticString begin, StaticString final,
        std::size_t N = std::get<1>(find_lca<begin, final>())>
    static constexpr auto path() noexcept {
        constexpr auto to_begin = child_path<begin>(true);
        constexpr auto to_final = child_path<final>(true);

        auto mismatch   = std::ranges::mismatch(to_begin, to_final);
        auto common_len = std::ranges::distance(to_begin.begin(), mismatch.in1);

        auto begin_to_lca = to_begin.size() - common_len;

        auto result = std::array<std::string_view, N> {};
        auto index  = 0;
        for (std::size_t i = 0; i < begin_to_lca; ++i) {
            auto side       = to_begin.size() - 1;
            result[index++] = to_begin[side - i];
        }
        for (std::size_t i = common_len; i < to_final.size(); ++i) {
            result[index++] = to_final[i];
        }
        return result;
    }

    template <StaticString name>
    constexpr static auto look_up(auto&& callback) noexcept {
        static_assert(Joint::contains<name>(), "Name was not found");
        std::ignore = find<name>([&]<class T> {
            using State = T::State;
            callback(tf::details::JointTransfroms<void, name, State>::state);
        });
    }
    template <StaticString name>
    constexpr static auto look_up(auto& dst) noexcept {
        look_up<name>([&](const auto& state) { dst = state; });
    }
    template <StaticString name, typename T>
    constexpr static auto look_up() noexcept {
        auto result = T {};
        look_up<name>([&](const auto& state) { result = state; });
        return result;
    }

    template <StaticString begin, StaticString final>
    constexpr static auto look_up(auto&& callback) noexcept {
        // ...
    }
    template <StaticString begin, StaticString final>
    constexpr static auto look_up(auto& dst) noexcept {
        look_up<begin, final>([&](const auto& state) { dst = state; });
    }
    template <StaticString begin, StaticString final, typename T>
    constexpr static auto look_up() noexcept {
        auto result = T {};
        look_up<begin, final>([&](const auto& state) { result = state; });
        return result;
    }

public:
    template <StaticString child>
    static constexpr auto impl_traversal_child(auto&& on_recursion) noexcept -> std::size_t {
        auto result    = std::size_t { 0 };
        auto recursion = [&]<class T>() {
            if (result) return;
            /*  */ if (child == T::name) {
                result = 1; // End Point
                on_recursion.template operator()<T>();
            } else if (auto sub = T::template impl_traversal_child<child>(on_recursion)) {
                result = 1 + sub; // Recursion Back
                on_recursion.template operator()<T>();
            }
        };
        (recursion.template operator()<Ts_>(), ...);
        return result;
    }
    template <StaticString child>
    static constexpr auto impl_traversal_child() noexcept {
        return impl_traversal_child<child>([]<typename T>() { });
    }
};

}

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
struct Link { };

template <StaticString name_, class State_ = tf::details::MonoState, class... Ts_>
struct Joint {
    using State = State_;

    constexpr explicit Joint(Link<name_, State_>, Ts_...) noexcept { }

    static constexpr std::size_t child_amount = sizeof...(Ts_);
    static constexpr std::size_t total_amount = 1 + (0 + ... + Ts_::total_amount);

    static constexpr auto name        = name_.view();
    static constexpr auto static_name = name_;

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
    template <StaticString a, StaticString b>
    static constexpr auto distance() noexcept {
        static_assert(a != b, "They are the same node");

        constexpr auto to_a = child_path<a>(true);
        constexpr auto to_b = child_path<b>(true);

        auto mismatch   = std::ranges::mismatch(to_a, to_b);
        auto common_len = std::ranges::distance(to_a.begin(), mismatch.in1);

        return to_a.size() + to_b.size() - 2 * common_len;
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
        static_assert(Joint::contains<child>(), "Child was not found");

        auto result = std::array<std::string_view, N> {};
        if (N == 0) return result;

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

        auto mismatch = std::ranges::mismatch(to_a, to_b);
        return std::make_tuple(*std::prev(mismatch.in1));
    }

    template <StaticString a, StaticString b>
    static constexpr auto distance_to_lca() noexcept {
        static_assert(a != b, "They are the same node");

        constexpr auto to_a = child_path<a>(true);
        constexpr auto to_b = child_path<b>(true);

        auto mismatch   = std::ranges::mismatch(to_a, to_b);
        auto common_len = std::ranges::distance(to_a.begin(), mismatch.in1);

        return std::make_tuple(to_a.size() - common_len, to_b.size() - common_len);
    }

    template <StaticString begin, StaticString final, std::size_t N = distance<begin, final>()>
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
    constexpr static auto get_state(auto&& callback) noexcept {
        static_assert(Joint::contains<name>(), "Name was not found");
        std::ignore = find<name>([&]<class T> {
            using State = T::State;
            callback(tf::details::JointTransfroms<Joint, name, State>::state);
        });
    }
    template <StaticString name>
    constexpr static auto get_state(auto& dst) noexcept {
        get_state<name>([&](const auto& state) { dst = state; });
    }
    template <StaticString name, typename T>
    constexpr static auto get_state() noexcept {
        auto result = T {};
        get_state<name>([&](const auto& state) { result = T { state }; });
        return result;
    }
    template <StaticString name>
    constexpr static auto set_state(const auto& _state) noexcept {
        get_state<name>([&](auto& state) { state = _state; });
    }

    template <StaticString name, typename T>
    using Transforms = tf::details::JointTransfroms<Joint, name, T>;

    /// @param: callback
    /// - name: std::string_view
    /// - state: SE3
    /// - is_begin: bool
    template <StaticString begin, StaticString final, class SE3>
    constexpr static auto look_up(auto&& callback) noexcept
        requires requires { callback(std::string_view {}, SE3 {}, bool {}); }
    {
        auto [begin_len, final_len] = distance_to_lca<begin, final>();
        // calculate tf from begin to lca
        impl_traversal_child<begin>([&]<class T>() {
            using State = typename T::State;
            if (begin_len-- > 0) {
                callback(T::name, Transforms<T::static_name, State>::state, true);
            }
        });
        // calculate tf from final to lca>
        impl_traversal_child<final>([&]<class T>() {
            using State = typename T::State;
            if (final_len-- > 0) {
                callback(T::name, Transforms<T::static_name, State>::state, false);
            }
        });
    }

    template <StaticString begin, StaticString final, class SE3>
        requires requires { SE3::Identity(); }
    constexpr static auto look_up() noexcept {
        auto lca_to_begin = SE3::Identity();
        auto lca_to_final = SE3::Identity();
        Joint::look_up<begin, final, SE3>([&](auto, auto se3, bool is_begin) {
            if (is_begin == true) {
                lca_to_begin = lca_to_begin * se3;
            }
            if (is_begin == false) {
                lca_to_final = lca_to_final * se3;
            }
        });
        return SE3 { lca_to_begin.inverse() * lca_to_final };
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

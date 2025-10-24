#pragma once
#include <rclcpp/exceptions/exceptions.hpp>

#include <expected>
#include <format>
#include <string>

namespace rmcs::utility {

template <class Node>
struct NodeAdapter {
    Node& node;

    explicit NodeAdapter(Node& node) noexcept
        : node { node } { }

    template <typename T>
    auto get_param(const std::string& name, T& target) noexcept
        -> std::expected<void, std::string> {
        try {
            if (!node.template get_parameter<T>(name, target)) {
                return std::unexpected {
                    std::format("Unable to find {}", name),
                };
            }
        } catch (rclcpp::exceptions::InvalidParameterTypeException& e) {
            return std::unexpected {
                std::format("Unexpected while getting {}: {}", name, e.what()),
            };
        } catch (const std::exception& e) {
            return std::unexpected {
                std::format("Catch unknown exception while getting {}: {}", name, e.what()),
            };
        }
        return {};
    }
};

template <class Data, typename Mem>
struct MemberMeta final {
    using D = Data;
    using M = Mem;
    using P = Mem Data::*;

    std::string_view meta_name;
    Mem Data::* mem_ptr;

    constexpr explicit MemberMeta(Mem Data::* mem_ptr, std::string_view id) noexcept
        : meta_name { id }
        , mem_ptr { mem_ptr } { }

    template <typename T>
    constexpr decltype(auto) extract_from(T&& data) const noexcept {
        return std::forward<T>(data).*mem_ptr;
    }
};

template <class Data, typename... Mem>
struct Serializable final {
    std::tuple<MemberMeta<Data, Mem>...> metas;

    constexpr explicit Serializable(MemberMeta<Data, Mem>... metas) noexcept
        : metas { std::tuple { metas... } } { }

    template <class Node>
    auto serialize(std::string_view prefix, Node& source, Data& target) const noexcept
        -> std::expected<void, std::string> {
        using Ret = std::expected<void, std::string>;

        auto adapter = NodeAdapter { source };

        const auto deserialize = [&]<typename T>(MemberMeta<Data, T> meta) -> Ret {
            auto& target_member = meta.extract_from(target);
            return adapter.get_param(std::format("{}{}", prefix, meta.meta_name), target_member);
        };
        const auto apply_function = [&]<typename... T>(MemberMeta<Data, T>... meta) {
            auto result = Ret {};
            std::ignore = ((result = deserialize(meta), result.has_value()) && ...);
            return result;
        };
        return std::apply(apply_function, metas);
    }

    auto make_printable_from(const Data& source) const noexcept -> std::string {
        auto result = std::string {};

        auto print_one = [&](const auto& meta) {
            using val_t = std::decay_t<decltype(meta.extract_from(source))>;

            if constexpr (std::formattable<val_t, char>) {
                result += std::format("{} = {}\n", meta.meta_name, meta.extract_from(source));
            } else {
                result += std::format("{} = ...\n", meta.meta_name);
            }
        };

        std::apply([&](const auto&... meta) { (print_one(meta), ...); }, metas);

        return result;
    }
};

}

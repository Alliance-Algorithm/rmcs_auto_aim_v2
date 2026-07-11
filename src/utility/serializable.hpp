#pragma once

#include <rclcpp/exceptions/exceptions.hpp>
#include <yaml-cpp/exceptions.h>

#include <concepts>
#include <expected>
#include <format>
#include <string>

namespace rmcs::util {

using integer_t = std::int64_t;
using double_t  = double;
using flag_t    = bool;
using string_t  = std::string;

// Extendable dispatch point
template <typename Source>
struct SerializableSource {
    static_assert(!std::is_same_v<Source, Source>,
        "No SerializableSource specialization for this source type. "
        "Create one via: template <> struct rmcs::util::SerializableSource<YourType> { ... };");

    template <typename T>
    static auto get(Source& source, const std::string& name, T& target) noexcept
        -> std::expected<void, std::string>;
};

namespace details {
    template <typename T>
    concept serializable_metas_trait =
        requires { typename std::tuple_size<decltype(T::metas)>::type; }
        && (std::tuple_size_v<decltype(T::metas)> != 0)
        && (std::tuple_size_v<decltype(T::metas)> % 2 == 0);

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

        template <class Source>
            requires((requires(Source& source, const std::string& name, Mem& target) {
                {
                    SerializableSource<std::remove_cvref_t<Source>>::get(source, name, target)
                } -> std::same_as<std::expected<void, std::string>>;
            }) && ...)
        auto serialize(std::string_view prefix, Source& source, Data& target) const noexcept
            -> std::expected<void, std::string> {
            using Ret = std::expected<void, std::string>;

            const auto deserialize = [&]<typename T>(MemberMeta<Data, T> meta) -> Ret {
                auto& target_member = meta.extract_from(target);
                return SerializableSource<std::remove_cvref_t<Source>>::get(source,
                    prefix.empty() ? std::string { meta.meta_name }
                                   : std::format("{}.{}", prefix, meta.meta_name),
                    target_member);
            };
            const auto apply_function = [&]<typename... T>(MemberMeta<Data, T>... meta) {
                auto result = Ret { };
                std::ignore = ((result = deserialize(meta), result.has_value()) && ...);
                return result;
            };
            return std::apply(apply_function, metas);
        }

        auto make_printable_from(const Data& source) const noexcept -> std::string {
            auto result = std::string { };

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

    template <typename Data, typename Source>
    concept serializable_source_trait = serializable_metas_trait<Data>
        && requires(Data& self, const Source& source, std::string_view prefix) {
               {
                   self.make_serializable(self.metas).serialize(prefix, source, self)
               } -> std::same_as<std::expected<void, std::string>>;
           };
}

struct Serializable {
    template <typename Metas, std::size_t... Idx>
    constexpr auto make_serializable_impl(Metas metas, std::index_sequence<Idx...>) {
        return details::Serializable {
            details::MemberMeta { std::get<Idx * 2>(metas), std::get<Idx * 2 + 1>(metas) }...,
        };
    }
    template <typename Metas>
    constexpr auto make_serializable(Metas metas) {
        constexpr auto N = std::tuple_size_v<Metas>;
        return make_serializable_impl(metas, std::make_index_sequence<N / 2> { });
    }

    template <typename T, typename Source>
        requires details::serializable_source_trait<T, Source>
    auto serialize(this T& self, std::string_view prefix, const Source& source) noexcept
        -> std::expected<void, std::string> {
        auto s = self.make_serializable(self.metas);
        return s.serialize(prefix, source, self);
    }

    template <class T, class Source>
        requires details::serializable_source_trait<T, Source>
    auto serialize(this T& self, const Source& source) noexcept {
        return self.serialize("", source);
    }

    auto printable(this auto& self) noexcept {
        auto s = self.make_serializable(self.metas);
        return s.make_printable_from(self);
    }
};
}

// Source-specific specializations

namespace YAML {
class Node;
}

template <typename Source>
    requires std::same_as<std::remove_cvref_t<Source>, YAML::Node>
struct rmcs::util::SerializableSource<Source> {
    template <typename T>
    static auto get(const Source& source, const std::string& name, T& target) noexcept
        -> std::expected<void, std::string> {
        try {
            if (!source[name]) {
                return std::unexpected {
                    std::format("Missing key '{}'", name),
                };
            }
            target = source[name].template as<T>();
        } catch (const YAML::TypedBadConversion<T>& e) {
            return std::unexpected {
                std::format("Type mismatch for '{}': {}", name, e.what()),
            };
        } catch (const std::exception& e) {
            return std::unexpected {
                std::format("Exception while reading '{}': {}", name, e.what()),
            };
        }
        return { };
    }
};

namespace rclcpp {
class Node;
}

template <typename Source>
    requires std::same_as<std::remove_cvref_t<Source>, rclcpp::Node>
struct rmcs::util::SerializableSource<Source> {
    template <typename T>
    static auto get(const Source& source, const std::string& name, T& target) noexcept
        -> std::expected<void, std::string> {
        try {
            if (!source.template get_parameter<T>(name, target)) {
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
        return { };
    }
};

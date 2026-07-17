#pragma once

#include <concepts>
#include <cstdint>
#include <expected>
#include <format>
#include <ranges>
#include <string>
#include <vector>

namespace rmcs::util {

struct Parameters {
    static auto share_location() noexcept -> std::string;
};

struct IParams {
    virtual ~IParams() = default;

    virtual auto contains(const std::string&) const -> bool = 0;

    virtual auto get_string(const std::string&) const -> std::string                    = 0;
    virtual auto get_string_array(const std::string&) const -> std::vector<std::string> = 0;

    virtual auto get_int64(const std::string&) const -> std::int64_t                    = 0;
    virtual auto get_int64_array(const std::string&) const -> std::vector<std::int64_t> = 0;

    virtual auto get_bool(const std::string&) const -> bool                    = 0;
    virtual auto get_bool_array(const std::string&) const -> std::vector<bool> = 0;

    virtual auto get_double(const std::string&) const -> double                    = 0;
    virtual auto get_double_array(const std::string&) const -> std::vector<double> = 0;

    virtual auto get_uint8_array(const std::string&) const -> std::vector<std::uint8_t> = 0;

    template <typename T>
    auto get(const std::string& name) const -> T {
        constexpr auto kIsBool   = std::same_as<T, bool>;
        constexpr auto kIsString = std::constructible_from<T, const std::string&>
            || std::assignable_from<T&, const std::string&>
            || std::convertible_to<const std::string&, T>;
        constexpr auto kIsDouble = std::floating_point<T>
            && (std::constructible_from<T, double> || std::assignable_from<T&, double>
                || std::convertible_to<double, T>);
        constexpr auto kIsInt = std::integral<T> && !kIsBool
            && (std::constructible_from<T, std::int64_t> || std::assignable_from<T&, std::int64_t>
                || std::convertible_to<std::int64_t, T>);

        /*^^*/ if constexpr (kIsBool) {
            return get_bool(name);
        } else if constexpr (kIsString) {
            return T { get_string(name) };
        } else if constexpr (kIsDouble) {
            return static_cast<T>(get_double(name));
        } else if constexpr (kIsInt) {
            return static_cast<T>(get_int64(name));
        } else if constexpr (std::ranges::range<T>) {
            constexpr auto kIsBoolArray   = std::same_as<T, std::vector<bool>>;
            constexpr auto kIsStringArray = std::same_as<T, std::vector<std::string>>;
            constexpr auto kIsDoubleArray = std::same_as<T, std::vector<double>>;
            constexpr auto kIsIntArray    = std::same_as<T, std::vector<std::int64_t>>;
            constexpr auto kIsUint8Array  = std::same_as<T, std::vector<std::uint8_t>>;

            /*^^*/ if constexpr (kIsBoolArray) {
                return T { get_bool_array(name) };
            } else if constexpr (kIsStringArray) {
                return T { get_string_array(name) };
            } else if constexpr (kIsDoubleArray) {
                return T { get_double_array(name) };
            } else if constexpr (kIsIntArray) {
                return T { get_int64_array(name) };
            } else if constexpr (kIsUint8Array) {
                return T { get_uint8_array(name) };
            } else {
                static_assert(false, "Unsupported array type for IParams::get<T>");
            }
        } else {
            static_assert(false, "Unsupported type for IParams::get<T>");
        }
    }
};

template <typename Source>
struct SerializableSource;

template <typename Source>
    requires std::derived_from<std::remove_cvref_t<Source>, IParams>
struct SerializableSource<Source> {
    template <typename T>
    static auto get(const Source& source, const std::string& name, T& target) noexcept
        -> std::expected<void, std::string> {
        try {
            target = source.template get<T>(name);
        } catch (const std::exception& e) {
            return std::unexpected {
                std::format("Exception while getting parameter '{}': {}", name, e.what()),
            };
        }
        return { };
    }
};

}

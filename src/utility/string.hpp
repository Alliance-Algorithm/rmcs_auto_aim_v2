#pragma once

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <string_view>

namespace rmcs::util {

[[nodiscard]] inline auto trim(std::string_view data) noexcept -> std::string_view {
    while (!data.empty() && std::isspace(static_cast<unsigned char>(data.front())) != 0)
        data.remove_prefix(1);
    while (!data.empty() && std::isspace(static_cast<unsigned char>(data.back())) != 0)
        data.remove_suffix(1);
    return data;
}

[[nodiscard]] inline auto ascii_iequals(std::string_view lhs, std::string_view rhs) noexcept
    -> bool {
    auto lower = [](char ch) {
        return static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    };
    return std::ranges::equal(lhs, rhs, std::ranges::equal_to { }, lower, lower);
}

template <std::size_t N>
struct StaticString {
    char data[N];

    static constexpr auto length() -> std::size_t { return N; }

    // NOLINTBEGIN(google-explicit-constructor)
    constexpr StaticString(const char (&s)[N]) noexcept {
        for (std::size_t i = 0; i < N; ++i)
            data[i] = s[i];
    }
    // NOLINTEND(google-explicit-constructor)

    constexpr explicit operator std::string_view() const noexcept { return data; }

    constexpr auto view() const noexcept { return std::string_view { data }; }

    constexpr bool operator==(const std::string_view& o) const noexcept { return o == view(); }

    template <std::size_t M>
    constexpr bool operator==(const StaticString<M>& o) const noexcept {
        if constexpr (N != M) {
            return false;
        } else {
            return view() == o.view();
        }
    }
};
template <std::size_t N>
StaticString(const char (&)[N]) -> StaticString<N>;

template <StaticString String>
struct Named {
    static constexpr auto kView = String.view();
    static constexpr auto kData = String.data;
};

}

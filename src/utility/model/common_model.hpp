#pragma once
#include "utility/string.hpp"

#include <algorithm>
#include <array>
#include <ranges>
#include <stdexcept>
#include <string_view>

#include <openvino/runtime/core.hpp>

namespace rmcs {

inline auto kRealTimePerformanceMode =
    ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY);

/// @brief:
/// POD 结构体，用于语义化地设置各个维度的值，比如：
/// ```
/// auto dimensions = Dimensions{ .W = 100, .H = 100 };
/// ```
struct Dimensions {
    using Value = ov::Dimension::value_type;

    Value N = 1;
    Value C = 3;
    Value W = 0;
    Value H = 0;

    constexpr auto at(char dimension) const -> Value {
        switch (dimension) {
        case 'N':
            return N;
        case 'C':
            return C;
        case 'W':
            return W;
        case 'H':
            return H;
        default:
            throw std::runtime_error("Wrong dimension char, valid: N, C, W, H");
        }
    }
};

///
/// @brief:
/// 模型布局，用于语义化生成 layout, shape 等数据结构
///
struct TensorLayout {
private:
    std::array<char, 5> chars { '\0', '\0', '\0', '\0', '\0' };

public:
    static constexpr auto is_valid_dimension(char dimension) noexcept {
        return dimension == 'N' || dimension == 'C' || dimension == 'W' || dimension == 'H';
    }
    template <std::ranges::input_range Range>
    static constexpr auto has_unique_dimensions(const Range& parsed) noexcept {
        auto has_unique_dimensions = true;
        for (auto it = std::ranges::begin(parsed); it != std::ranges::end(parsed); ++it) {
            if (std::ranges::find(std::next(it), std::ranges::end(parsed), *it)
                != std::ranges::end(parsed)) {
                has_unique_dimensions = false;
            }
        }
        return has_unique_dimensions;
    }
    static constexpr auto is_valid_description(std::string_view description) noexcept -> bool {
        return std::ranges::all_of(description, is_valid_dimension)
            && has_unique_dimensions(description);
    }

    template <util::StaticString description>
    static consteval auto from() noexcept -> TensorLayout {
        static_assert(description.length() == 5, "Layout description must be exactly 4 characters");

        constexpr auto parsed = std::array<char, 4> {
            description.data[0],
            description.data[1],
            description.data[2],
            description.data[3],
        };
        static_assert(std::ranges::all_of(parsed, is_valid_dimension),
            "The layout description only supports N/C/W/H");
        static_assert(has_unique_dimensions(parsed),
            "The layout description must not contain duplicate dimensions");

        return TensorLayout { std::string_view { parsed.data(), 4 } };
    }

public:
    constexpr explicit TensorLayout(std::string_view description) {
        if (description.size() == 5 && description[4] == '\0') {
            description.remove_suffix(1);
        }
        if (description.size() != 4) {
            throw std::invalid_argument { "Layout description must be exactly 4 characters" };
        }
        if (!is_valid_description(description)) {
            throw std::invalid_argument { "Invalid layout description" };
        }
        std::ranges::copy_n(description.begin(), 4, chars.begin());
    }

    constexpr auto layout() const noexcept { return ov::Layout { chars.data() }; }

    constexpr auto partial_shape(const Dimensions& dimensions) const noexcept {
        return ov::PartialShape {
            dimensions.at(chars[0]),
            dimensions.at(chars[1]),
            dimensions.at(chars[2]),
            dimensions.at(chars[3]),
        };
    }
    constexpr auto shape(const Dimensions& dimensions) const noexcept {
        return ov::Shape { {
            static_cast<std::size_t>(dimensions.at(chars[0])),
            static_cast<std::size_t>(dimensions.at(chars[1])),
            static_cast<std::size_t>(dimensions.at(chars[2])),
            static_cast<std::size_t>(dimensions.at(chars[3])),
        } };
    }
};
}

#pragma once
#include "utility/robot/armor.hpp"
#include <opencv2/core/types.hpp>

namespace rmcs {

template <typename precision_type = float>
struct ArmorInferResult {
    using Point = cv::Point_<precision_type>;
    using Rect  = cv::Rect_<precision_type>;

    //(0,1) 顶左，(2,3) 底左，(4,5) 底右，(6,7) 顶右
    struct Corners {
        precision_type lt_x;
        precision_type lt_y;
        precision_type lb_x;
        precision_type lb_y;
        precision_type rb_x;
        precision_type rb_y;
        precision_type rt_x;
        precision_type rt_y;
        auto lt() const noexcept { return Point { lt_x, lt_y }; }
        auto rb() const noexcept { return Point { rb_x, rb_y }; }
        auto rt() const noexcept { return Point { rt_x, rt_y }; }
        auto lb() const noexcept { return Point { lb_x, lb_y }; }
    } corners;

    precision_type confidence;

    struct Color {
        precision_type blue;
        precision_type red;
        precision_type dark;
        precision_type mix;
    } color;

    struct Role {
        precision_type sentry;
        precision_type hero;
        precision_type engineer;
        precision_type infantry_3;
        precision_type infantry_4;
        precision_type infantry_5;
        precision_type outpost;
        precision_type base;
        precision_type nothing;
    } role;

    auto unsafe_from(std::span<const precision_type> raw) noexcept {
        static_assert(std::is_trivially_copyable_v<ArmorInferResult>);
        std::memcpy(this, raw.data(), sizeof(ArmorInferResult));
    }

    auto bounding_rect() const noexcept {
        const std::array xs { corners.lt_x, corners.rt_x, corners.rb_x, corners.lb_x };
        const std::array ys { corners.lt_y, corners.rt_y, corners.rb_y, corners.lb_y };
        const auto min_x = std::ranges::min(xs);
        const auto max_x = std::ranges::max(xs);
        const auto min_y = std::ranges::min(ys);
        const auto max_y = std::ranges::max(ys);
        return Rect { min_x, min_y, max_x - min_x, max_y - min_y };
    }

    auto bl() const noexcept { return corners.lb(); }
    auto br() const noexcept { return corners.rb(); }
    auto tl() const noexcept { return corners.lt(); }
    auto tr() const noexcept { return corners.rt(); }

    auto scale_corners(precision_type scaling) noexcept {
        corners.lb_x *= scaling;
        corners.lb_y *= scaling;
        corners.lt_x *= scaling;
        corners.lt_y *= scaling;
        corners.rb_x *= scaling;
        corners.rb_y *= scaling;
        corners.rt_x *= scaling;
        corners.rt_y *= scaling;
    }

    auto armor_color() const noexcept {
        constexpr static std::array enums {
            ArmorColor::RED,
            ArmorColor::BLUE,
            ArmorColor::DARK,
            ArmorColor::MIX,
        };
        const std::array colors {
            color.red,
            color.blue,
            color.dark,
            color.mix,
        };
        return max_element(enums, colors);
    }

    auto armor_genre() const noexcept {
        constexpr static std::array enums {
            ArmorGenre::UNKNOWN,
            ArmorGenre::HERO,
            ArmorGenre::ENGINEER,
            ArmorGenre::INFANTRY_3,
            ArmorGenre::INFANTRY_4,
            ArmorGenre::INFANTRY_5,
            ArmorGenre::SENTRY,
            ArmorGenre::OUTPOST,
            ArmorGenre::BASE,
        };
        const std::array genres {
            role.nothing,
            role.hero,
            role.engineer,
            role.infantry_3,
            role.infantry_4,
            role.infantry_5,
            role.sentry,
            role.outpost,
            role.base,
        };
        return max_element(enums, genres);
    }

    constexpr static auto length() noexcept {
        return sizeof(ArmorInferResult) / sizeof(precision_type);
    }

    template <typename T1, typename T2, std::size_t N>
    constexpr static auto max_element(
        const std::array<T1, N>& a1, const std::array<T2, N>& a2) noexcept {
        const auto it  = std::ranges::max_element(a2);
        const auto idx = std::ranges::distance(a2.begin(), it);
        return a1[idx];
    }
};

}

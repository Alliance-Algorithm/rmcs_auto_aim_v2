#pragma once
#include "utility/robot/armor.hpp"
#include <opencv2/core/types.hpp>

namespace rmcs {

template <class data_type>
struct InferResultAdapter {
    using precision_type = typename data_type::precision_type;
    using rectangle_type = cv::Rect_<precision_type>;
    using point_type     = cv::Point_<precision_type>;

    data_type data;

    auto bl() const noexcept -> point_type { return { data.corners.lb_x, data.corners.lb_y }; }
    auto br() const noexcept -> point_type { return { data.corners.rb_x, data.corners.rb_y }; }
    auto tl() const noexcept -> point_type { return { data.corners.lt_x, data.corners.lt_y }; }
    auto tr() const noexcept -> point_type { return { data.corners.rt_x, data.corners.rt_y }; }

    // Confidence
    auto confidence() noexcept -> precision_type& { return data.confidence; }
    auto confidence() const noexcept -> const precision_type& { return data.confidence; }

    // Color
    auto blue() const noexcept -> precision_type { return data.color.blue; }
    auto red() const noexcept -> precision_type { return data.color.red; }
    auto dark() const noexcept -> precision_type { return data.color.dark; }
    auto mix() const noexcept -> precision_type { return data.color.mix; }

    // Genre
    auto sentry() const noexcept -> precision_type { return data.genre.sentry; }
    auto hero() const noexcept -> precision_type { return data.genre.hero; }
    auto engineer() const noexcept -> precision_type { return data.genre.engineer; }
    auto infantry_3() const noexcept -> precision_type { return data.genre.infantry_3; }
    auto infantry_4() const noexcept -> precision_type { return data.genre.infantry_4; }
    auto infantry_5() const noexcept -> precision_type { return data.genre.infantry_5; }
    auto outpost() const noexcept -> precision_type { return data.genre.outpost; }
    auto base_small() const noexcept -> precision_type {
        if constexpr (requires { data.genre.base_small; }) return data.genre.base_small;
        else return { 0 };
    }
    auto base_large() const noexcept -> precision_type {
        if constexpr (requires { data.genre.base_large; }) return data.genre.base_large;
        else return data.genre.base;
    }
    auto unknown() const noexcept -> precision_type {
        if constexpr (requires { data.genre.nothing; }) return data.genre.nothing;
        else return { 0 };
    }

    // Util
    auto unsafe_from(std::span<const precision_type> raw) noexcept -> void {
        static_assert(std::is_trivially_copyable_v<data_type>);
        if (raw.size() < length()) return;
        std::memcpy(&data, raw.data(), sizeof(data_type));
    }

    auto bounding_rect() const noexcept -> rectangle_type {
        const std::array _bl { bl().x, bl().y };
        const std::array _br { br().x, br().y };
        const std::array _tl { tl().x, tl().y };
        const std::array _tr { tr().x, tr().y };
        const std::array xs { _tl[0], _tr[0], _br[0], _bl[0] };
        const std::array ys { _tl[1], _tr[1], _br[1], _bl[1] };
        const auto min_x = std::ranges::min(xs);
        const auto max_x = std::ranges::max(xs);
        const auto min_y = std::ranges::min(ys);
        const auto max_y = std::ranges::max(ys);
        return { min_x, min_y, max_x - min_x, max_y - min_y };
    }

    auto scale_corners(precision_type scaling) noexcept -> void {
        data.corners.lb_x *= scaling;
        data.corners.lb_y *= scaling;
        data.corners.lt_x *= scaling;
        data.corners.lt_y *= scaling;
        data.corners.rb_x *= scaling;
        data.corners.rb_y *= scaling;
        data.corners.rt_x *= scaling;
        data.corners.rt_y *= scaling;
    }
    auto armor_color() const noexcept -> ArmorColor {
        constexpr static std::array enums {
            ArmorColor::RED,
            ArmorColor::BLUE,
            ArmorColor::DARK,
            ArmorColor::MIX,
        };
        const std::array colors {
            red(),
            blue(),
            dark(),
            mix(),
        };
        return max_element(enums, colors);
    }

    auto armor_genre() const noexcept -> ArmorGenre {
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
            ArmorGenre::BASE,
        };
        const std::array genres {
            unknown(),
            hero(),
            engineer(),
            infantry_3(),
            infantry_4(),
            infantry_5(),
            sentry(),
            outpost(),
            base_small(),
            base_large(),
        };
        return max_element(enums, genres);
    }

    constexpr static auto length() noexcept -> std::size_t {
        return sizeof(data_type) / sizeof(precision_type);
    }

    template <typename T1, typename T2, std::size_t N>
    constexpr static auto max_element(
        const std::array<T1, N>& a1, const std::array<T2, N>& a2) noexcept -> T1 {
        const auto it  = std::ranges::max_element(a2);
        const auto idx = std::ranges::distance(a2.begin(), it);
        return a1[idx];
    }
};

}

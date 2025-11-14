#pragma once
#include <opencv2/core/types.hpp>

namespace rmcs {

template <typename precision_type = float>
struct ArmorDetection {
    using Point = cv::Point_<precision_type>;
    using Rect  = cv::Rect_<precision_type>;

    struct Corners {
        precision_type lt_x;
        precision_type lt_y;
        precision_type rb_x;
        precision_type rb_y;
        precision_type rt_x;
        precision_type rt_y;
        precision_type lb_x;
        precision_type lb_y;
        auto lt() const noexcept { return Point { lt_x, lt_y }; }
        auto rb() const noexcept { return Point { rb_x, rb_y }; }
        auto rt() const noexcept { return Point { rt_x, rt_y }; }
        auto lb() const noexcept { return Point { lb_x, lb_y }; }
    } corners;

    precision_type confidence;

    struct Color {
        precision_type red;
        precision_type blue;
        precision_type dark;
        precision_type mix;
    } color;

    struct Role {
        precision_type hero;
        precision_type engineer;
        precision_type infantry_3;
        precision_type infantry_4;
        precision_type infantry_5;
        precision_type sentry;
        precision_type outpost;
        precision_type base;
        precision_type nothing;
    } role;

    auto unsafe_from(std::span<const precision_type> raw) noexcept {
        static_assert(std::is_trivially_copyable_v<ArmorDetection>);
        std::memcpy(this, raw.data(), sizeof(ArmorDetection));
    }

    auto bounding_rect() const noexcept {
        using std::max;
        using std::min;

        const auto min_x = min(min(corners.lt_x, corners.rt_x), min(corners.rb_x, corners.lb_x));
        const auto max_x = max(max(corners.lt_x, corners.rt_x), max(corners.rb_x, corners.lb_x));
        const auto min_y = min(min(corners.lt_y, corners.rt_y), min(corners.rb_y, corners.lb_y));
        const auto max_y = max(max(corners.lt_y, corners.rt_y), max(corners.rb_y, corners.lb_y));

        const auto w = max_x - min_x;
        const auto h = max_y - min_y;

        return Rect { min_x, min_y, w, h };
    }

    auto lt() const noexcept { return corners.lt(); }
    auto rb() const noexcept { return corners.rb(); }
    auto rt() const noexcept { return corners.rt(); }
    auto lb() const noexcept { return corners.lb(); }

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

    constexpr static auto length() noexcept {
        return sizeof(ArmorDetection) / sizeof(precision_type);
    }
};

}

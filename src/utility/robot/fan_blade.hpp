#pragma once

#include "utility/math/linear.hpp"
#include <array>
#include <cstdint>
#include <opencv2/core/types.hpp>

namespace rmcs {
enum class FanBladeColor : std::uint8_t { DARK, RED, BLUE };
struct FanBlade2D {
    cv::Point2f center;                 // 扇页中心
    std::array<cv::Point2f, 4> corners; // 四个点从左上角开始逆时针
    FanBladeColor color;
    double angle; // 扇叶中心与R标连线与R标水平线的夹角
};

struct FanBlade3D {
    bool is_target;
    FanBladeColor color;

    Translation R_mark_position;
    Translation position;
    Orientation orientation;
};

}

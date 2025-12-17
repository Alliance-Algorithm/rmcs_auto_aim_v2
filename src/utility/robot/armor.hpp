#pragma once
#include "utility/math/linear.hpp"
#include "utility/math/point.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"
#include <generator>
#include <opencv2/core/types.hpp>

namespace rmcs {

enum class ArmorColor : std::uint8_t { DARK, RED, BLUE, MIX };
constexpr auto get_enum_name(ArmorColor color) noexcept {
    constexpr std::array details { "DARK", "RED", "BLUE", "MIX" };
    return details[std::to_underlying(color)];
}
constexpr auto armor_color2camp_color(ArmorColor color) -> CampColor {
    if (color == ArmorColor::BLUE) return CampColor::BLUE;
    if (color == ArmorColor::RED) return CampColor::RED;
    return CampColor::UNKNOWN;
};

constexpr auto camp_color2armor_color(CampColor color) -> ArmorColor {
    if (color == CampColor::BLUE) return ArmorColor::BLUE;
    if (color == CampColor::RED) return ArmorColor::RED;
    return ArmorColor::MIX;
};

enum class ArmorShape : bool { LARGE, SMALL };
constexpr auto get_enum_name(ArmorShape shape) noexcept {
    constexpr std::array details { "LARGE", "SMALL" };
    return details[std::to_underlying(shape)];
};

using ArmorGenre = DeviceId;
constexpr auto get_enum_name(ArmorGenre genre) noexcept { return rmcs::to_string(genre); }

struct Armor2D {
    ArmorGenre genre;
    ArmorColor color;
    ArmorShape shape;

    double confidence;

    cv::Point2f tl;
    cv::Point2f tr;
    cv::Point2f br;
    cv::Point2f bl;

    cv::Point2f center;

    auto corners() const noexcept -> std::generator<const cv::Point2f&> {
        co_yield tl;
        co_yield tr;
        co_yield br;
        co_yield bl;
    }
};

struct Armor3D {
    ArmorGenre genre;
    ArmorColor color;
    int id;

    Translation translation;
    Orientation orientation;
};

struct Armor { };
using Armors = std::vector<Armor>;

constexpr auto kLightBarHeight  = 0.056;
constexpr auto kLargeArmorWidth = 0.23;
constexpr auto kSmallArmorWidth = 0.135;

constexpr std::array<Point3d, 4> kLargeArmorShapeOpenCV {
    Point3d { -0.5 * kLargeArmorWidth, -0.5 * kLightBarHeight, 0.0 }, // Top-left
    Point3d { +0.5 * kLargeArmorWidth, -0.5 * kLightBarHeight, 0.0 }, // Top-right
    Point3d { +0.5 * kLargeArmorWidth, +0.5 * kLightBarHeight, 0.0 }, // Bottom-right
    Point3d { -0.5 * kLargeArmorWidth, +0.5 * kLightBarHeight, 0.0 }  // Bottom-left
};

constexpr std::array<Point3d, 4> kSmallArmorShapeOpenCV {
    Point3d { -0.5 * kSmallArmorWidth, -0.5 * kLightBarHeight, 0.0 }, // Top-left
    Point3d { +0.5 * kSmallArmorWidth, -0.5 * kLightBarHeight, 0.0 }, // Top-right
    Point3d { +0.5 * kSmallArmorWidth, +0.5 * kLightBarHeight, 0.0 }, // Bottom-right
    Point3d { -0.5 * kSmallArmorWidth, +0.5 * kLightBarHeight, 0.0 }  // Bottom-left
};

constexpr std::array<Point3d, 4> kLargeArmorShapeRos {
    Point3d { 0.0, +0.5 * kLargeArmorWidth, +0.5 * kLightBarHeight }, // Top-left
    Point3d { 0.0, -0.5 * kLargeArmorWidth, -0.5 * kLightBarHeight }, // Bottom-right
    Point3d { 0.0, -0.5 * kLargeArmorWidth, +0.5 * kLightBarHeight }, // Top-right
    Point3d { 0.0, +0.5 * kLargeArmorWidth, -0.5 * kLightBarHeight }  // Bottom-left
};
constexpr std::array<Point3d, 4> kSmallArmorShapeRos {
    Point3d { 0.0, +0.5 * kSmallArmorWidth, +0.5 * kLightBarHeight }, // Top-left
    Point3d { 0.0, -0.5 * kSmallArmorWidth, -0.5 * kLightBarHeight }, // Bottom-right
    Point3d { 0.0, -0.5 * kSmallArmorWidth, +0.5 * kLightBarHeight }, // Top-right
    Point3d { 0.0, +0.5 * kSmallArmorWidth, -0.5 * kLightBarHeight }  // Bottom-left
};
}

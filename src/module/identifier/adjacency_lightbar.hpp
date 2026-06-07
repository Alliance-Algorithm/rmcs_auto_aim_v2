#pragma once

#include "utility/image/image.hpp"
#include "utility/math/camera.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"

#include <optional>

namespace rmcs {

class AdjacencyLightbarFinder {
    RMCS_PIMPL_DEFINITION(AdjacencyLightbarFinder)

public:
    struct Lightbar {
        Point2d upper;
        Point2d lower;
        bool is_right { false };
        bool is_upper { false };
    };

    auto set_camera_feature(const util::CameraFeature& feature) -> void;

    auto set_armor_thickness(double thickness) -> void;

    auto find(const Image& image, const Armor2d& armor2d, const Armor3d& armor3d)
        -> std::optional<Lightbar>;

    // 绘制上一次 find 使用的 roi
    auto draw_roi(Image& image) -> void;

    auto draw_lightbar(Image& image) -> void;
};

}

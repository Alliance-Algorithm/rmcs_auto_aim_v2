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
    struct Result {
        Lightbar2ds found;
        Point2d center;
        Lightbar2ds predicted_near;
        Lightbar2ds predicted_away;
        std::vector<cv::Rect2i> areas;
    };

    auto set_camera_feature(const util::CameraFeature& feature) -> void;

    auto set_armor_thickness(double thickness) -> void;

    auto find(const Image& image, const Armor2d& armor2d, const Armor3d& armor3d)
        -> std::optional<Result>;
};

}

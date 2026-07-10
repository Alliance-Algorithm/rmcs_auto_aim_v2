#pragma once
#include "utility/math/camera.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/rune.hpp"

#include <opencv2/core/mat.hpp>

namespace rmcs {

class RuneDetector {
public:
    struct Config {
        util::CameraFeature cam;
        CampColor color = CampColor::BLUE;

        double max_distance = 5.;
        double min_distance = 1.;

        double max_perspective = 60.;

        int red_diff_threshold    = 30;
        int blue_diff_threshold   = 30;
        int min_channel_threshold = 60;

        double match_threshold  = 0.2;
        double active_threshold = 0.1;
    } config;

    struct Elements {
        std::vector<RuneBullseye> bullseyes;
        std::vector<RuneIcon> icons;
    };

    auto detect(const cv::Mat&) const -> Elements;
};

}

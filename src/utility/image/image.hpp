#pragma once
#include "utility/clock.hpp"

#include <opencv2/core/mat.hpp>

namespace rmcs {

struct Image {
    cv::Mat mat;
    Timestamp timestamp { };
};

}

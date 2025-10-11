#pragma once

#include "image.hpp"
#include <opencv2/core/mat.hpp>

namespace rmcs {

struct Image::Details {
    cv::Mat mat;
};

}

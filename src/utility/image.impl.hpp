#pragma once

#include "image.hpp"
#include <opencv2/core/mat.hpp>

namespace rmcs {

struct Image::Details {
    cv::Mat mat;

    auto set_mat(cv::Mat mat) noexcept -> void {
        //
        this->mat = std::move(mat);
    }
    decltype(auto) get_mat(this auto& self) noexcept {
        //
        return self.mat;
    }
};

}

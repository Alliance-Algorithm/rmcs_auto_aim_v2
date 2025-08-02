#pragma once

#include <chrono>
#include <opencv2/core/mat.hpp>

namespace rmcs {

template <class T>
concept concept_capturer = requires(T& capturer) {
    { capturer.read(std::chrono::seconds(5)) } -> std::same_as<cv::Mat>;
};
template <class T>
concept concept_identifier = requires {
    true; //
};
template <class T>
concept concept_tracker = requires {
    true; //
};

} // namespace rmcs

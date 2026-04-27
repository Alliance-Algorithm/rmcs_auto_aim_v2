#pragma once
#include "utility/pimpl.hpp"

#include <chrono>
#include <string>

#include <opencv2/core/mat.hpp>

namespace rmcs {

class ImageRecorder {
    RMCS_PIMPL_DEFINITION(ImageRecorder)

public:
    auto set_saving_location(const std::string&) -> void;
    auto set_framerate(std::size_t) -> void;
    auto set_auto_save(bool) -> void;
    auto set_max_history_count(std::size_t) -> void;
    auto set_max_recording_duration(std::chrono::seconds) -> void;
    auto set_min_recording_duration(std::chrono::seconds) -> void;

    auto write_frame(const cv::Mat&) -> void;
    auto stop() -> void;

    [[nodiscard]] auto recording() const -> bool;
    [[nodiscard]] auto current_file_path() const -> std::string;
    [[nodiscard]] auto last_saved_path() const -> std::string;
};

}

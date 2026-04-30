#include "utility/image/recorder.hpp"

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core/mat.hpp>

#include <gtest/gtest.h>

namespace {

auto list_recordings(const std::filesystem::path& directory) -> std::vector<std::filesystem::path> {
    auto recordings = std::vector<std::filesystem::path> { };

    if (!std::filesystem::exists(directory)) {
        return recordings;
    }

    for (const auto& entry : std::filesystem::directory_iterator { directory }) {
        if (entry.is_regular_file() && entry.path().extension() == ".avi") {
            recordings.push_back(entry.path());
        }
    }

    return recordings;
}

class ImageRecorderTest : public ::testing::Test {
protected:
    auto SetUp() -> void override {
        base_directory = std::filesystem::temp_directory_path() / "rmcs_image_recorder_test";
        std::filesystem::remove_all(base_directory);
        std::filesystem::create_directories(base_directory);
    }

    auto TearDown() -> void override { std::filesystem::remove_all(base_directory); }

    std::filesystem::path base_directory = { };
};

TEST_F(ImageRecorderTest, saves_recording_when_duration_is_valid) {
    auto recorder = rmcs::ImageRecorder { };
    recorder.set_saving_location(base_directory.string());
    recorder.set_framerate(60);
    recorder.set_min_recording_duration(std::chrono::seconds { 1 });
    recorder.set_max_recording_duration(std::chrono::seconds { 5 });

    auto frame = cv::Mat { 32, 32, CV_8UC3, cv::Scalar { 10, 20, 30 } };

    recorder.write_frame(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds { 1100 });
    recorder.stop();

    const auto recordings = list_recordings(base_directory);
    ASSERT_EQ(recordings.size(), 1);
    EXPECT_GT(std::filesystem::file_size(recordings.front()), 0);
}

TEST_F(ImageRecorderTest, discards_recording_when_duration_is_too_short) {
    auto recorder = rmcs::ImageRecorder { };
    recorder.set_saving_location(base_directory.string());
    recorder.set_framerate(60);
    recorder.set_min_recording_duration(std::chrono::seconds { 2 });
    recorder.set_max_recording_duration(std::chrono::seconds { 5 });

    auto frame = cv::Mat { 32, 32, CV_8UC3, cv::Scalar { 10, 20, 30 } };

    recorder.write_frame(frame);
    recorder.stop();

    EXPECT_TRUE(list_recordings(base_directory).empty());
}

} // namespace

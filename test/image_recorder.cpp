#include "utility/image/recorder.hpp"

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core/mat.hpp>

#include <gtest/gtest.h>

namespace {

class VideoRecorderTest : public ::testing::Test {
protected:
    auto SetUp() -> void override {
        base_directory = std::filesystem::temp_directory_path() / "rmcs_video_recorder_test";
        std::filesystem::remove_all(base_directory);
        std::filesystem::create_directories(base_directory);
    }

    auto TearDown() -> void override { std::filesystem::remove_all(base_directory); }

    std::filesystem::path base_directory = { };
};

TEST_F(VideoRecorderTest, start_and_stop_recording) {
    auto recorder = rmcs::VideoRecorder { };
    recorder.update_config(rmcs::VideoRecorder::Config {
        .directories    = { base_directory.string() },
        .max_duration   = std::chrono::minutes { 1 },
        .max_videos_size = 1ull * 1024 * 1024 * 1024,
    });

    ASSERT_TRUE(recorder.start().has_value());

    auto frame = cv::Mat { 32, 32, CV_8UC3, cv::Scalar { 10, 20, 30 } };
    recorder.tick(frame);

    std::this_thread::sleep_for(std::chrono::milliseconds { 100 });
    recorder.tick(frame);

    recorder.stop();

    EXPECT_FALSE(recorder.recording());
}

TEST_F(VideoRecorderTest, start_fails_with_invalid_directory) {
    auto recorder = rmcs::VideoRecorder { };
    recorder.update_config(rmcs::VideoRecorder::Config {
        .directories    = { "/nonexistent/path/that/should/not/exist" },
        .max_duration   = std::chrono::minutes { 1 },
        .max_videos_size = 1ull * 1024 * 1024 * 1024,
    });

    auto result = recorder.start();
    EXPECT_FALSE(result.has_value());
}

TEST_F(VideoRecorderTest, filename_returns_nullopt_when_not_recording) {
    auto recorder = rmcs::VideoRecorder { };
    EXPECT_FALSE(recorder.filename().has_value());
}

} // namespace

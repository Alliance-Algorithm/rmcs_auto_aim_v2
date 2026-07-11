#pragma once

#include "utility/clock.hpp"
#include "utility/math/linear.hpp"

#include <memory>

#include <opencv2/core/mat.hpp>
#include <rmcs_msgs/camera_frame.hpp>

namespace rmcs {

class Image {
public:
    using Frame = rmcs_msgs::CameraFrame;

    explicit Image(std::shared_ptr<const Frame> frame) noexcept
        : frame_ { std::move(frame) }
        , mat_ { Frame::kHeight, Frame::kWidth, CV_8UC3,
            const_cast<std::byte*>(frame_->data.data()) } { }

    auto frame() const noexcept -> const std::shared_ptr<const Frame>& { return frame_; }

    auto mat() const noexcept -> const cv::Mat& { return mat_; }
    auto clone_mat() const -> cv::Mat { return mat_.clone(); }

    auto timestamp() const noexcept -> Timestamp { return frame_->exposure_timestamp; }
    auto imu_orientation() const noexcept -> Orientation {
        return Orientation { frame_->imu_snapshot };
    }

    auto reception_timestamp() const noexcept -> Timestamp {
        return frame_->image_reception_timestamp;
    }
    auto publish_timestamp() const noexcept -> Timestamp { return frame_->sync_publish_timestamp; }

private:
    std::shared_ptr<const Frame> frame_;
    cv::Mat mat_;
};

}

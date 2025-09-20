#pragma once

#include "stream_mode.hpp"
#include "utility/node.hpp"
#include <opencv2/core/mat.hpp>

namespace rmcs::module {

/// @brief
/// Use the gstreamer module built into opencv to stream video
/// transform node or armor plate
///
/// @note
/// depends, use apt to install them:
///      - gstreamer1.0-tools
///      - gstreamer1.0-plugins-base
///      - gstreamer1.0-plugins-good
///
class Streamer {
public:
    // Use node to log
    explicit Streamer(utility::Node&) noexcept;
    ~Streamer() noexcept;
    Streamer(const Streamer&)            = delete;
    Streamer& operator=(const Streamer&) = delete;

    // Real-time Transport Protocol using UDP
    struct RTP_UDP : public UdpConfig { };
    // Real-Time Streaming Protocol using UDP
    struct RTSP_UDP : public UdpConfig { };
    // Real-Time Streaming Protocol using TCP
    struct RTSP_TCP : public TcpConfig { };

    auto open(const RTP_UDP& config) -> bool;
    auto open(const RTSP_UDP& config) -> bool;
    auto open(const RTSP_TCP& config) -> bool;

    auto opened() const noexcept -> bool;

    /// @brief
    /// Pushes a cv::Mat image into the queue and then streaming.
    ///
    /// @param image
    /// Image to be pushed into the queue. Although ideally this would be moved,
    /// boost::lockfree::spsc_queue does not support move semantics. Therefore, the image is passed
    /// as const reference, resulting in a shallow copy.
    ///
    /// @note
    /// cv::Mat uses internal reference counting, so the image data is shared across copies unless
    /// explicitly cloned. And only one thread is allowed to push image to the tool.
    ///
    /// @return
    /// true, if the push operation is successful.
    ///
    auto send(const cv::Mat& image) noexcept -> bool;

    /// @brief
    /// Generates a synthetic test image with a gradually changing hue.
    ///
    /// @param hue
    /// Reference to a hue value used to generate the image. The hue is incremented after each call,
    /// cycling within the HSV hue range [0, 179].
    ///
    /// @return
    /// A cv::Mat containing the generated BGR image.
    ///
    auto generate_test_image(uint8_t& hue) const noexcept -> cv::Mat;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

}

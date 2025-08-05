#pragma once
#include "utility/node.hpp"
#include <opencv2/core/mat.hpp>

namespace rmcs::module {

/// @brief
/// Use the gstreamer module built into opencv to stream video and visualize some 3d data like
/// transform node or armor plate
///
/// @note
/// depends:
///      - gstreamer1.0-tools
///
/// some tools may be useful:
///      - gstreamer1.0-plugins-base
///      - gstreamer1.0-plugins-good
///      - gstreamer1.0-tools
///      - gstreamer1.0-plugins-ugly
///
class Visualization {
public:
    // Use node to log
    explicit Visualization(utility::Node&) noexcept;
    ~Visualization() noexcept;
    Visualization(const Visualization&)            = delete;
    Visualization& operator=(const Visualization&) = delete;

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
    auto streaming(const cv::Mat& image) noexcept -> bool;

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

    /// @brief
    /// Block process and generate a video stream with gradient color
    ///
    /// @note
    /// target: 127.0.0.1:5000
    /// width : 640
    /// height: 480
    /// fps   : 30
    /// Just for test
    /// You can use vlc to play this stream:
    ///
    ///     v=0
    ///     m=video 5000 RTP/AVP 26
    ///     c=IN IP4 127.0.0.1
    ///     a=rtpmap:26 JPEG/90000
    ///
    /// Create streaming.sdp with the above content and open it with vlc
    ///
    static auto block_and_test(utility::Node& node) noexcept -> void;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rmcs::debug

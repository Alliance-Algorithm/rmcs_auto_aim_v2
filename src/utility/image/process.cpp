#include "process.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace rmcs::util {

auto extract_channel(const cv::Mat& src, CampColor camp, cv::Mat& dst) -> void {
    constexpr auto kRedDiffThreshold  = 40;
    constexpr auto kBlueDiffThreshold = 40;

    constexpr auto kRedPurityThreshold  = 20; // R - G
    constexpr auto kBluePurityThreshold = 20; // B - G

    constexpr auto kMinChannelThreshold = 80;

    auto channels = std::vector<cv::Mat> { };
    cv::split(src, channels);

    auto diff_color    = cv::Mat { };
    auto diff_color_u8 = cv::Mat { };
    auto diff_purity   = cv::Mat { };
    auto purity_u8     = cv::Mat { };
    auto mask          = cv::Mat { };
    auto min_mask      = cv::Mat { };

    if (camp == CampColor::RED) {
        // 1. 颜色差值：R - B
        cv::subtract(channels[2], channels[0], diff_color, cv::noArray(), CV_16S);
        diff_color.convertTo(diff_color_u8, CV_8U);
        cv::threshold(diff_color_u8, mask, kRedDiffThreshold, 255, cv::THRESH_BINARY);

        // 2. 纯度过滤：R - G（白色 R≈G，通不过；红色 R>>G，通过）
        cv::subtract(channels[2], channels[1], diff_purity, cv::noArray(), CV_16S);
        diff_purity.convertTo(purity_u8, CV_8U);
        cv::threshold(purity_u8, min_mask, kRedPurityThreshold, 255, cv::THRESH_BINARY);

        cv::bitwise_and(mask, min_mask, mask);

        // 3. 亮度检查：R > threshold
        cv::threshold(channels[2], min_mask, kMinChannelThreshold, 255, cv::THRESH_BINARY);
        cv::bitwise_and(mask, min_mask, dst);

    } else if (camp == CampColor::BLUE) {
        // 1. 颜色差值：B - R
        cv::subtract(channels[0], channels[2], diff_color, cv::noArray(), CV_16S);
        diff_color.convertTo(diff_color_u8, CV_8U);
        cv::threshold(diff_color_u8, mask, kBlueDiffThreshold, 255, cv::THRESH_BINARY);

        // 2. 纯度过滤：B - G（白色 B≈G，通不过；蓝色 B>>G，通过）
        cv::subtract(channels[0], channels[1], diff_purity, cv::noArray(), CV_16S);
        diff_purity.convertTo(purity_u8, CV_8U);
        cv::threshold(purity_u8, min_mask, kBluePurityThreshold, 255, cv::THRESH_BINARY);

        cv::bitwise_and(mask, min_mask, mask);

        // 3. 亮度检查：B > threshold
        cv::threshold(channels[0], min_mask, kMinChannelThreshold, 255, cv::THRESH_BINARY);
        cv::bitwise_and(mask, min_mask, dst);

    } else {
        dst = cv::Mat { };
    }
    cv::dilate(dst, dst, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)),
        cv::Point(-1, -1), 1);
}

}

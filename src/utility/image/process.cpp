#include "process.hpp"
#include <opencv2/imgproc.hpp>

namespace rmcs::util {

auto extract_channel(const cv::Mat& src, CampColor camp, cv::Mat& dst) -> void {
    constexpr auto kRedDiffThreshold    = 20;
    constexpr auto kBlueDiffThreshold   = 20;
    constexpr auto kRedPurityThreshold  = 10;
    constexpr auto kBluePurityThreshold = 10;
    constexpr auto kMinChannelThreshold = 40;

    // 次通道比例上限：单色 LED 的次通道是主通道的串扰，比例 < 50%
    // 红色 LED: G <= R/2  →  R >= 2G
    // 蓝色 LED: G <= B/2  →  B >= 2G
    constexpr auto kSubChannelRatio = 2;

    constexpr auto kCloseKernelSize = 3;
    constexpr auto kCloseIterations = 1;

    auto channels = std::vector<cv::Mat> { };
    cv::split(src, channels);

    auto diff_color    = cv::Mat { };
    auto diff_color_u8 = cv::Mat { };
    auto diff_purity   = cv::Mat { };
    auto purity_u8     = cv::Mat { };
    auto mask          = cv::Mat { };
    auto min_mask      = cv::Mat { };

    /*^^*/ if (camp == CampColor::RED) {
        cv::subtract(channels[2], channels[0], diff_color, cv::noArray(), CV_16S);
        diff_color.convertTo(diff_color_u8, CV_8U);
        cv::threshold(diff_color_u8, mask, kRedDiffThreshold, 255, cv::THRESH_BINARY);

        cv::subtract(channels[2], channels[1], diff_purity, cv::noArray(), CV_16S);
        diff_purity.convertTo(purity_u8, CV_8U);
        cv::threshold(purity_u8, min_mask, kRedPurityThreshold, 255, cv::THRESH_BINARY);
        cv::bitwise_and(mask, min_mask, mask);

        cv::threshold(channels[2], min_mask, kMinChannelThreshold, 255, cv::THRESH_BINARY);
        cv::bitwise_and(mask, min_mask, mask);

        // R >= 2G（次通道不超过主通道的一半）
        auto scaled_main = cv::Mat { };
        auto scaled_sub  = cv::Mat { };
        cv::multiply(channels[2], 1, scaled_main, 1, CV_16S); // R * 1
        cv::multiply(channels[1], kSubChannelRatio, scaled_sub, 1, CV_16S); // G * 2
        cv::compare(scaled_main, scaled_sub, min_mask, cv::CMP_GE); // R >= 2G
        cv::bitwise_and(mask, min_mask, dst);

    } else if (camp == CampColor::BLUE) {
        cv::subtract(channels[0], channels[2], diff_color, cv::noArray(), CV_16S);
        diff_color.convertTo(diff_color_u8, CV_8U);
        cv::threshold(diff_color_u8, mask, kBlueDiffThreshold, 255, cv::THRESH_BINARY);

        cv::subtract(channels[0], channels[1], diff_purity, cv::noArray(), CV_16S);
        diff_purity.convertTo(purity_u8, CV_8U);
        cv::threshold(purity_u8, min_mask, kBluePurityThreshold, 255, cv::THRESH_BINARY);
        cv::bitwise_and(mask, min_mask, mask);

        cv::threshold(channels[0], min_mask, kMinChannelThreshold, 255, cv::THRESH_BINARY);
        cv::bitwise_and(mask, min_mask, mask);

        // B >= 2G
        auto scaled_main = cv::Mat { };
        auto scaled_sub  = cv::Mat { };
        cv::multiply(channels[0], 1, scaled_main, 1, CV_16S); // B * 1
        cv::multiply(channels[1], kSubChannelRatio, scaled_sub, 1, CV_16S); // G * 2
        cv::compare(scaled_main, scaled_sub, min_mask, cv::CMP_GE); // B >= 2G
        cv::bitwise_and(mask, min_mask, dst);

    } else {
        dst = cv::Mat { };
        return;
    }

    auto kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kCloseKernelSize, kCloseKernelSize));
    cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), kCloseIterations);
}

}

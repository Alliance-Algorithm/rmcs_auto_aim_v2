#pragma once
#include "utility/math/camera.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/rune.hpp"

#include <opencv2/core/mat.hpp>

namespace rmcs {

class RuneDetector {
public:
    struct Config {
        util::CameraFeature cam;
        CampColor color = CampColor::BLUE;

        double max_distance = 5.0;
        double min_distance = 1.7;

        double max_perspective = 60.;

        // 激活靶心判定阈值，使用 PVD（Peak-Valley Depth）指标：
        // PVD ≈ 0 表示角度投影各向均匀，为未激活圆环；
        // PVD 较大表示存在清晰的 4 个峰/谷结构，为已激活靶心。
        // PVD ≥ 0.2 时按 4 臂端点提取，PVD < 0.2 时退化为圆环处理。
        double active_threshold = 0.2;

        // R 标模板匹配阈值。采用骨架拓扑特征匹配，值越大表示 ROI 越像 R 标；
        // 分数 < 阈值时直接丢弃该候选。
        double match_threshold = 0.50;
    } config;

    struct Elements {
        std::vector<RuneBullseye> bullseyes;
        std::vector<RuneIcon> icons;
    };

    auto detect(const cv::Mat&) const -> Elements;
};

}

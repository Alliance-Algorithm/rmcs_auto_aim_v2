#pragma once
#include "utility/robot/color.hpp"
#include "utility/robot/rune.hpp"

#include <opencv2/core/mat.hpp>

namespace rmcs {

struct RuneFinder {
    struct Input {
        cv::Mat image;
        CampColor color;

        int red_diff_threshold    = 30;
        int blue_diff_threshold   = 30;
        int min_channel_threshold = 60;

        double min_area       = 60.0;
        double max_area       = 20000.0;
        double min_side_ratio = 0.99;
        double max_side_ratio = 1.55;
        double min_area_ratio = 0.50;
        double max_area_ratio = 1.20;
        double min_peri_ratio = 0.35;
        double max_peri_ratio = 1.60;

        double active_arm_min_area         = 2000.0;
        double active_arm_max_area         = 20000.0;
        double active_arm_min_aspect_ratio = 0.8;
        double active_arm_max_aspect_ratio = 5.0;
        double active_arm_min_area_ratio   = 0.1;
        double active_arm_max_area_ratio   = 0.6;
        double active_arm_min_hull_ratio   = 2.0;
        double active_arm_max_hull_ratio   = 5.0;
        double active_arm_min_peri_ratio   = 0.75;
        double active_arm_max_peri_ratio   = 1.25;
        double active_center_min_distance  = 30.0;

        double active_target_min_area                      = 60.0;
        double active_target_max_area                      = 20000.0;
        double active_target_min_side_ratio                = 0.99;
        double active_target_max_side_ratio                = 1.55;
        double active_target_min_area_ratio                = 0.80;
        double active_target_max_area_ratio                = 1.20;
        double active_target_min_peri_ratio                = 0.35;
        double active_target_max_peri_ratio                = 0.80;
        double active_target_min_convex_area_ratio         = 0.30;
        double active_target_max_direct_children           = 4.0;
        double active_target_max_tenring_sub_area_ratio    = 0.30;
        double active_target_min_concentric_sub_area_ratio = 0.70;

        double gap_min_area_ratio      = 0.025;
        double gap_max_area_ratio      = 0.20;
        double gap_min_side_ratio      = 1.55;
        double gap_max_side_ratio      = 8.0;
        double gap_circle_radius_ratio = 0.7037;
        double gap_min_distance_ratio  = 0.50;
        double gap_max_distance_ratio  = 0.80;
        double gap_min_open_angle      = 60.0;
        double gap_max_open_angle      = 100.0;

        double center_min_area              = 100.0;
        double center_max_area              = 1000.0;
        double center_min_side_ratio        = 0.3;
        double center_max_side_ratio        = 2.5;
        double center_min_roundness         = 0.2;
        double center_max_roundness         = 0.9;
        double center_max_sub_area_ratio    = 0.2;
        double center_min_convex_area_ratio = 0.9;
        double center_max_defect_area_ratio = 0.3;
        double center_min_area_for_ratio    = 20.0;
        double center_concentricity_ratio   = 0.08;
    } input;

    struct Result {
        Point2d icon;
        std::vector<RunePage> pages;
    } result;

    auto solve() noexcept -> bool;
};

}

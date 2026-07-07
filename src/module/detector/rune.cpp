#include "rune.hpp"

#include <opencv2/imgproc.hpp>

#include <array>
#include <cmath>
#include <numbers>
#include <unordered_set>
#include <vector>

namespace rmcs {

namespace {

    const auto kVacancyPoint = cv::Point2f { -1.0f, -1.0f };

    constexpr auto kGapDirectionAngleLow  = 70.0;
    constexpr auto kGapDirectionAngleHigh = 110.0;

    constexpr auto kEpsilon = 1e-6;

    inline auto get_unit_vector(const cv::Point2f& point) -> cv::Point2f {
        const auto norm = std::hypot(point.x, point.y);
        if (norm < kEpsilon) return cv::Point2f { 0.0f, 0.0f };
        return cv::Point2f { point.x / norm, point.y / norm };
    }

    inline auto get_vector_angle(const cv::Point2f& left, const cv::Point2f& right) -> double {
        const auto dot   = left.x * right.x + left.y * right.y;
        const auto cross = left.x * right.y - left.y * right.x;
        return std::atan2(std::abs(cross), dot) * 180.0 / std::numbers::pi;
    }

    inline auto get_distance(const cv::Point2f& left, const cv::Point2f& right) -> double {
        return std::hypot(left.x - right.x, left.y - right.y);
    }

    inline auto to_point2d(const cv::Point2f& point) -> Point2d {
        return Point2d { static_cast<double>(point.x), static_cast<double>(point.y) };
    }

    auto make_binary_image(const cv::Mat& image, const RuneFinder::Input& input) -> cv::Mat {
        if (image.empty()) return cv::Mat { };

        cv::Mat mask;
        {
            std::vector<cv::Mat> channels;
            cv::split(image, channels);

            cv::Mat difference;
            cv::Mat difference_u8;
            cv::Mat minimum_mask;

            if (input.color == CampColor::RED) {
                cv::subtract(channels[2], channels[0], difference, cv::noArray(), CV_16S);
                difference.convertTo(difference_u8, CV_8U);
                cv::threshold(
                    difference_u8, mask, input.red_diff_threshold, 255, cv::THRESH_BINARY);

                cv::threshold(
                    channels[2], minimum_mask, input.min_channel_threshold, 255, cv::THRESH_BINARY);
            } else if (input.color == CampColor::BLUE) {
                cv::subtract(channels[0], channels[2], difference, cv::noArray(), CV_16S);
                difference.convertTo(difference_u8, CV_8U);
                cv::threshold(
                    difference_u8, mask, input.blue_diff_threshold, 255, cv::THRESH_BINARY);

                cv::threshold(
                    channels[0], minimum_mask, input.min_channel_threshold, 255, cv::THRESH_BINARY);
            } else {
                return cv::Mat { };
            }

            cv::bitwise_and(mask, minimum_mask, mask);
        }

        return mask;
    }

    auto get_valid_contour_indices(const std::vector<std::vector<cv::Point>>& contours,
        double min_area, double max_area) -> std::unordered_set<size_t> {
        std::unordered_set<size_t> result;
        for (size_t index = 0; index < contours.size(); ++index) {
            const auto area = cv::contourArea(contours[index]);
            if (area >= min_area && area <= max_area) {
                result.insert(index);
            }
        }
        return result;
    }

    auto is_contour_valid(size_t index, const std::unordered_set<size_t>& valid) -> bool {
        return valid.find(index) != valid.end();
    }

    auto get_direct_sub_contour_indices(
        const std::vector<cv::Vec4i>& hierarchy, size_t index, std::vector<int>& result) -> void {
        if (index >= hierarchy.size()) return;

        auto child = hierarchy[index][2];
        while (child != -1) {
            result.push_back(child);
            child = hierarchy[child][0];
        }
    }

    auto get_all_sub_contour_indices(
        const std::vector<cv::Vec4i>& hierarchy, size_t index, std::vector<int>& result) -> void {
        std::vector<int> direct_children;
        get_direct_sub_contour_indices(hierarchy, index, direct_children);
        for (const auto child : direct_children) {
            result.push_back(child);
            get_all_sub_contour_indices(hierarchy, child, result);
        }
    }

    auto check_ellipse_shape(const std::vector<cv::Point>& contour, const RuneFinder::Input& input)
        -> std::optional<cv::RotatedRect> {
        if (contour.size() < 6) return std::nullopt;

        const auto area = cv::contourArea(contour);
        if (area < input.min_area || area > input.max_area) return std::nullopt;

        const auto ellipse = cv::fitEllipse(contour);
        const auto width   = std::max(ellipse.size.width, ellipse.size.height);
        const auto height  = std::min(ellipse.size.width, ellipse.size.height);
        if (height <= 0) return std::nullopt;

        const auto aspect_ratio = width / height;
        if (aspect_ratio < input.min_side_ratio || aspect_ratio > input.max_side_ratio)
            return std::nullopt;

        const auto ellipse_area = width * height * std::numbers::pi / 4.0;
        const auto area_ratio   = area / ellipse_area;
        if (area_ratio < input.min_area_ratio || area_ratio > input.max_area_ratio)
            return std::nullopt;

        const auto perimeter         = cv::arcLength(contour, true);
        const auto ellipse_perimeter = std::numbers::pi
            * (3.0 * (width + height) - std::sqrt((3.0 * width + height) * (width + 3.0 * height)));
        if (ellipse_perimeter <= 0) return std::nullopt;

        const auto perimeter_ratio = perimeter / ellipse_perimeter;
        if (perimeter_ratio < input.min_peri_ratio || perimeter_ratio > input.max_peri_ratio)
            return std::nullopt;

        return ellipse;
    }

    auto check_arm_shape(const std::vector<cv::Point>& contour, const RuneFinder::Input& input)
        -> std::optional<cv::RotatedRect> {
        if (contour.size() < 6) return std::nullopt;

        const auto area = cv::contourArea(contour);
        if (area < input.active_arm_min_area || area > input.active_arm_max_area)
            return std::nullopt;

        const auto ellipse = cv::fitEllipse(contour);
        const auto width   = std::max(ellipse.size.width, ellipse.size.height);
        const auto height  = std::min(ellipse.size.width, ellipse.size.height);
        if (height <= 0) return std::nullopt;

        const auto aspect_ratio = width / height;
        if (aspect_ratio < input.active_arm_min_aspect_ratio
            || aspect_ratio > input.active_arm_max_aspect_ratio)
            return std::nullopt;

        const auto ellipse_area = width * height * std::numbers::pi / 4.0;
        const auto area_ratio   = area / ellipse_area;
        if (area_ratio < input.active_arm_min_area_ratio
            || area_ratio > input.active_arm_max_area_ratio)
            return std::nullopt;

        auto hull = std::vector<cv::Point> { };
        cv::convexHull(contour, hull);
        const auto hull_area = cv::contourArea(hull);
        if (hull_area <= 0) return std::nullopt;

        const auto hull_ratio = hull_area / area;
        if (hull_ratio < input.active_arm_min_hull_ratio
            || hull_ratio > input.active_arm_max_hull_ratio)
            return std::nullopt;

        const auto perimeter         = cv::arcLength(contour, true);
        const auto ellipse_perimeter = std::numbers::pi
            * (3.0 * (width + height) - std::sqrt((3.0 * width + height) * (width + 3.0 * height)));
        if (ellipse_perimeter <= 0) return std::nullopt;

        const auto perimeter_ratio = perimeter / ellipse_perimeter;
        if (perimeter_ratio < input.active_arm_min_peri_ratio
            || perimeter_ratio > input.active_arm_max_peri_ratio)
            return std::nullopt;

        return ellipse;
    }

    auto check_active_target(const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<cv::Vec4i>& hierarchy, size_t index, const RuneFinder::Input& input)
        -> std::optional<cv::RotatedRect> {
        const auto& contour = contours[index];
        if (contour.size() < 6) return std::nullopt;

        const auto area = cv::contourArea(contour);
        if (area < input.active_target_min_area || area > input.active_target_max_area)
            return std::nullopt;

        const auto ellipse = cv::fitEllipse(contour);
        const auto width   = std::max(ellipse.size.width, ellipse.size.height);
        const auto height  = std::min(ellipse.size.width, ellipse.size.height);
        if (height <= 0) return std::nullopt;

        const auto aspect_ratio = width / height;
        if (aspect_ratio < input.active_target_min_side_ratio
            || aspect_ratio > input.active_target_max_side_ratio)
            return std::nullopt;

        const auto ellipse_area = width * height * std::numbers::pi / 4.0;
        const auto area_ratio   = area / ellipse_area;
        if (area_ratio < input.active_target_min_area_ratio
            || area_ratio > input.active_target_max_area_ratio)
            return std::nullopt;

        const auto perimeter         = cv::arcLength(contour, true);
        const auto ellipse_perimeter = std::numbers::pi
            * (3.0 * (width + height) - std::sqrt((3.0 * width + height) * (width + 3.0 * height)));
        if (ellipse_perimeter <= 0) return std::nullopt;

        const auto perimeter_ratio = perimeter / ellipse_perimeter;
        if (perimeter_ratio < input.active_target_min_peri_ratio
            || perimeter_ratio > input.active_target_max_peri_ratio)
            return std::nullopt;

        auto hull = std::vector<cv::Point> { };
        cv::convexHull(contour, hull);
        const auto hull_area = cv::contourArea(hull);
        if (hull_area <= 0) return std::nullopt;

        const auto convexity = area / hull_area;
        if (convexity < input.active_target_min_convex_area_ratio) return std::nullopt;

        auto direct_children = std::vector<int> { };
        get_direct_sub_contour_indices(hierarchy, index, direct_children);
        if (direct_children.size()
            >= static_cast<std::size_t>(input.active_target_max_direct_children))
            return std::nullopt;

        auto all_sub_indices = std::vector<int> { };
        get_all_sub_contour_indices(hierarchy, index, all_sub_indices);
        if (all_sub_indices.empty()) return std::nullopt;

        double total_sub_area = 0.0;
        double max_sub_area   = 0.0;
        size_t max_sub_idx    = 0;
        for (const auto sub_idx : all_sub_indices) {
            if (sub_idx < 0 || sub_idx >= static_cast<int>(contours.size())) continue;
            const auto sub_area = cv::contourArea(contours[sub_idx]);
            total_sub_area += sub_area;
            if (sub_area > max_sub_area) {
                max_sub_area = sub_area;
                max_sub_idx  = static_cast<size_t>(sub_idx);
            }
        }

        const auto ten_ring_ratio = total_sub_area / area;
        if (ten_ring_ratio > input.active_target_max_tenring_sub_area_ratio) {
            if (max_sub_area / area < input.active_target_min_concentric_sub_area_ratio)
                return std::nullopt;

            const auto sub_ellipse = cv::fitEllipse(contours[max_sub_idx]);
            const auto sub_width   = std::max(sub_ellipse.size.width, sub_ellipse.size.height);
            const auto sub_height  = std::min(sub_ellipse.size.width, sub_ellipse.size.height);
            if (sub_height <= 0) return std::nullopt;
            const auto sub_aspect_ratio = sub_width / sub_height;
            if (sub_aspect_ratio < input.min_side_ratio || sub_aspect_ratio > input.max_side_ratio)
                return std::nullopt;
        } else if (all_sub_indices.size() < 3) {
            return std::nullopt;
        }

        return ellipse;
    }

    auto find_center(const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<cv::Vec4i>& hierarchy, const std::unordered_set<size_t>& valid,
        const RuneFinder::Input& input) -> std::optional<cv::Point2f> {
        for (size_t index = 0; index < contours.size(); ++index) {
            if (!is_contour_valid(index, valid)) continue;
            if (hierarchy[index][3] != -1) continue;
            if (contours[index].size() < 6) continue;

            const auto area = cv::contourArea(contours[index]);
            if (area < input.center_min_area || area > input.center_max_area) continue;

            const auto ellipse = cv::fitEllipse(contours[index]);
            const auto width   = std::max(ellipse.size.width, ellipse.size.height);
            const auto height  = std::min(ellipse.size.width, ellipse.size.height);
            if (height <= 0) continue;

            const auto aspect_ratio = width / height;
            if (aspect_ratio < input.center_min_side_ratio
                || aspect_ratio > input.center_max_side_ratio)
                continue;

            const auto perimeter = cv::arcLength(contours[index], true);
            if (perimeter <= 0) continue;

            const auto roundness = 4.0 * std::numbers::pi * area / (perimeter * perimeter);
            if (roundness < input.center_min_roundness || roundness > input.center_max_roundness)
                continue;

            if (area <= input.center_min_area_for_ratio) return ellipse.center;

            std::vector<int> sub_indices;
            get_direct_sub_contour_indices(hierarchy, index, sub_indices);

            double total_sub_area = 0.0;
            for (const auto sub_index : sub_indices) {
                if (sub_index < 0 || sub_index >= static_cast<int>(contours.size())) continue;
                if (!is_contour_valid(sub_index, valid)) continue;
                total_sub_area += cv::contourArea(contours[sub_index]);
            }
            if (total_sub_area / area > input.center_max_sub_area_ratio) continue;

            const auto hull      = cv::Mat { contours[index] };
            const auto hull_area = cv::contourArea(hull);
            if (hull_area > 0 && area / hull_area < input.center_min_convex_area_ratio) continue;

            return ellipse.center;
        }

        return std::nullopt;
    }

    auto find_active_pages(const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<cv::Vec4i>& hierarchy, const std::unordered_set<size_t>& valid,
        const std::optional<cv::Point2f>& rune_center, const RuneFinder::Input& input)
        -> std::vector<RunePage> {
        struct Candidate {
            size_t index;
            RunePage page;
            bool is_target;
        };
        std::vector<Candidate> candidates;

        for (size_t index = 0; index < contours.size(); ++index) {
            if (!is_contour_valid(index, valid)) continue;
            if (hierarchy[index][3] != -1) continue;

            auto maybe_page = check_arm_shape(contours[index], input);
            bool is_target  = false;
            if (!maybe_page) {
                maybe_page = check_active_target(contours, hierarchy, index, input);
                is_target  = maybe_page.has_value();
            }
            if (!maybe_page) continue;

            // 跳过神符中心（R 标）自身
            if (rune_center) {
                const auto center_distance = get_distance(
                    cv::Point2f { rune_center->x, rune_center->y }, maybe_page->center);
                if (center_distance < input.active_center_min_distance) continue;
            }

            RunePage page;
            page.active = true;
            page.center = to_point2d(maybe_page->center);
            candidates.push_back(Candidate { index, page, is_target });
        }

        // 过滤掉被其他已激活页轮廓包含的误检
        std::vector<Candidate> filtered;
        for (const auto& candidate : candidates) {
            bool is_inside_another = false;
            for (const auto& other : candidates) {
                if (candidate.index == other.index) continue;
                if (cv::pointPolygonTest(contours[other.index],
                        cv::Point2f { static_cast<float>(candidate.page.center.x),
                            static_cast<float>(candidate.page.center.y) },
                        false)
                    > 0) {
                    is_inside_another = true;
                    break;
                }
            }
            if (!is_inside_another) filtered.push_back(candidate);
        }

        // 过滤掉与已激活靶心（圆环）相邻的灯臂误检
        std::vector<RunePage> result;
        for (const auto& candidate : filtered) {
            if (!candidate.is_target) {
                bool near_target = false;
                for (const auto& other : filtered) {
                    if (!other.is_target || candidate.index == other.index) continue;
                    const auto target_ellipse = cv::fitEllipse(contours[other.index]);
                    const auto target_size =
                        std::max(target_ellipse.size.width, target_ellipse.size.height);
                    const auto center_distance =
                        get_distance(cv::Point2f { static_cast<float>(candidate.page.center.x),
                                         static_cast<float>(candidate.page.center.y) },
                            target_ellipse.center);
                    if (center_distance < target_size * 1.5) {
                        near_target = true;
                        break;
                    }
                }
                if (near_target) continue;
            }
            result.push_back(candidate.page);
        }

        return result;
    }

    auto get_ellipse_correction_matrix(const cv::RotatedRect& ellipse, float& radius)
        -> cv::Matx33f {
        const auto rotation_affine = cv::getRotationMatrix2D(ellipse.center, ellipse.angle, 1.0);
        const auto rotation_matrix = cv::Matx33f {
            static_cast<float>(rotation_affine.at<double>(0, 0)),
            static_cast<float>(rotation_affine.at<double>(0, 1)),
            static_cast<float>(rotation_affine.at<double>(0, 2)),
            static_cast<float>(rotation_affine.at<double>(1, 0)),
            static_cast<float>(rotation_affine.at<double>(1, 1)),
            static_cast<float>(rotation_affine.at<double>(1, 2)),
            0.0f,
            0.0f,
            1.0f,
        };

        const auto rotation_inverse   = rotation_matrix.inv();
        const auto translation_matrix = cv::Matx33f {
            1.0,
            0.0,
            -ellipse.center.x,
            0.0,
            1.0,
            -ellipse.center.y,
            0.0,
            0.0,
            1.0,
        };
        const auto translation_inverse = translation_matrix.inv();

        const auto length       = (ellipse.size.width + ellipse.size.height) * 2.0f;
        const auto scale_matrix = cv::Matx33f {
            length / ellipse.size.width,
            0.0,
            0.0,
            0.0,
            length / ellipse.size.height,
            0.0,
            0.0,
            0.0,
            1.0,
        };
        radius = length / 2.0f;

        return rotation_inverse * translation_inverse * scale_matrix * translation_matrix
            * rotation_matrix;
    }

    auto transform_contour(const std::vector<cv::Point>& contour, const cv::Matx33f& matrix)
        -> std::vector<cv::Point2f> {
        std::vector<cv::Point2f> result;
        result.reserve(contour.size());
        for (const auto& point : contour) {
            auto vector =
                cv::Matx31f { static_cast<float>(point.x), static_cast<float>(point.y), 1.0f };
            const auto transformed = matrix * vector;
            result.emplace_back(transformed(0), transformed(1));
        }
        return result;
    }

    auto get_left_right_index(const std::vector<cv::Point2f>& contour, const cv::Point2f& center)
        -> std::pair<int, int> {
        int left_index    = 0;
        int right_index   = 0;
        auto left_vector  = contour[0] - center;
        auto right_vector = contour[0] - center;

        for (size_t index = 1; index < contour.size(); ++index) {
            const auto current_vector = contour[index] - center;
            if (left_vector.cross(current_vector) < 0) {
                left_vector = current_vector;
                left_index  = static_cast<int>(index);
            }
            if (right_vector.cross(current_vector) > 0) {
                right_vector = current_vector;
                right_index  = static_cast<int>(index);
            }
        }

        return { left_index, right_index };
    }

    struct Gap {
        cv::Point2f center;
        cv::Point2f left_corner;
        cv::Point2f right_corner;
        bool valid = true;
    };

    auto make_gap(const std::vector<cv::Point>& gap_contour, const cv::RotatedRect& outer_ellipse,
        const RuneFinder::Input& input) -> std::optional<Gap> {
        if (gap_contour.size() < 10) return std::nullopt;

        const auto area = cv::contourArea(gap_contour);
        const auto outer_area =
            outer_ellipse.size.width * outer_ellipse.size.height * std::numbers::pi / 4.0;
        if (area / outer_area < input.gap_min_area_ratio
            || area / outer_area > input.gap_max_area_ratio)
            return std::nullopt;

        const auto gap_ellipse = cv::fitEllipse(gap_contour);
        const auto width       = std::max(gap_ellipse.size.width, gap_ellipse.size.height);
        const auto height      = std::min(gap_ellipse.size.width, gap_ellipse.size.height);
        if (height <= 0) return std::nullopt;

        const auto aspect_ratio = width / height;
        if (aspect_ratio < input.gap_min_side_ratio || aspect_ratio > input.gap_max_side_ratio)
            return std::nullopt;

        float radius                         = 0.0f;
        const auto correction_matrix         = get_ellipse_correction_matrix(outer_ellipse, radius);
        const auto correction_matrix_inverse = correction_matrix.inv();

        const auto corrected_contour = transform_contour(gap_contour, correction_matrix);
        const auto corrected_ellipse = cv::fitEllipse(corrected_contour);

        const auto outer_center    = cv::Point2f { outer_ellipse.center.x, outer_ellipse.center.y };
        const auto center_distance = get_distance(corrected_ellipse.center, outer_center);
        if (center_distance > radius * input.gap_max_distance_ratio
            || center_distance < radius * input.gap_min_distance_ratio)
            return std::nullopt;

        const auto [left_index, right_index] =
            get_left_right_index(corrected_contour, outer_center);

        const auto left_unit  = get_unit_vector(corrected_contour[left_index] - outer_center);
        const auto right_unit = get_unit_vector(corrected_contour[right_index] - outer_center);

        const auto radius_scale = static_cast<float>(input.gap_circle_radius_ratio * radius);
        const auto left_corner  = outer_center + left_unit * radius_scale;
        const auto right_corner = outer_center + right_unit * radius_scale;
        const auto gap_center =
            outer_center + get_unit_vector(corrected_ellipse.center - outer_center) * radius_scale;

        const auto opening_angle =
            get_vector_angle(left_corner - outer_center, right_corner - outer_center);
        if (opening_angle < input.gap_min_open_angle || opening_angle > input.gap_max_open_angle)
            return std::nullopt;

        const auto gap_direction   = get_unit_vector(right_corner - left_corner);
        const auto center_to_gap   = get_unit_vector(gap_center - outer_center);
        const auto direction_angle = get_vector_angle(gap_direction, center_to_gap);
        if (direction_angle < kGapDirectionAngleLow || direction_angle > kGapDirectionAngleHigh)
            return std::nullopt;

        const auto inverse = [&](const cv::Point2f& point) {
            auto vector =
                cv::Matx31f { static_cast<float>(point.x), static_cast<float>(point.y), 1.0f };
            const auto transformed = correction_matrix_inverse * vector;
            return cv::Point2f { transformed(0), transformed(1) };
        };

        Gap gap;
        gap.left_corner  = inverse(left_corner);
        gap.right_corner = inverse(right_corner);
        gap.center       = inverse(gap_center);
        gap.valid        = true;
        return gap;
    }

    auto find_gaps(const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<cv::Vec4i>& hierarchy, const std::unordered_set<size_t>& valid,
        size_t outer_index, const cv::RotatedRect& outer_ellipse, const RuneFinder::Input& input)
        -> std::vector<Gap> {
        std::vector<Gap> result;

        std::vector<int> sub_indices;
        get_direct_sub_contour_indices(hierarchy, outer_index, sub_indices);

        for (const auto sub_index : sub_indices) {
            if (sub_index < 0 || sub_index >= static_cast<int>(contours.size())) continue;
            if (!is_contour_valid(sub_index, valid)) continue;
            if (contours[sub_index].size() < 10) continue;

            const auto maybe_gap = make_gap(contours[sub_index], outer_ellipse, input);
            if (maybe_gap) {
                result.push_back(*maybe_gap);
            }
        }

        return result;
    }

    auto sort_gaps(const std::vector<Gap>& gaps, const cv::Point2f& page_center,
        const cv::Point2f& rune_center) -> std::array<Gap, 4> {
        std::array<Gap, 4> result;
        for (auto& gap : result) {
            gap.center       = kVacancyPoint;
            gap.left_corner  = kVacancyPoint;
            gap.right_corner = kVacancyPoint;
            gap.valid        = false;
        }

        const auto center_to_rune = get_unit_vector(rune_center - page_center);

        for (const auto& gap : gaps) {
            const auto center_to_gap = get_unit_vector(gap.center - page_center);
            const auto dot =
                center_to_rune.x * center_to_gap.x + center_to_rune.y * center_to_gap.y;
            const auto cross =
                center_to_rune.x * center_to_gap.y - center_to_rune.y * center_to_gap.x;

            size_t index = 0;
            if (dot < 0) {
                index = (cross > 0) ? 0 : 3; // 左上 : 右上
            } else {
                index = (cross > 0) ? 1 : 2; // 左下 : 右下
            }
            result[index]       = gap;
            result[index].valid = true;
        }

        return result;
    }

    auto make_crosshair_gaps(const std::vector<cv::Point>& gap_contour,
        const cv::RotatedRect& outer_ellipse, const RuneFinder::Input& input) -> std::vector<Gap> {
        const auto gap_area = cv::contourArea(gap_contour);
        const auto outer_area =
            outer_ellipse.size.width * outer_ellipse.size.height * std::numbers::pi / 4.0;
        const auto gap_area_ratio = gap_area / outer_area;
        if (gap_area_ratio < 0.05 || gap_area_ratio > 0.40) return { };

        const auto gap_ellipse = cv::fitEllipse(gap_contour);
        const auto gap_width   = std::max(gap_ellipse.size.width, gap_ellipse.size.height);
        const auto gap_height  = std::min(gap_ellipse.size.width, gap_ellipse.size.height);
        if (gap_height <= 0) return { };
        if (gap_width / gap_height > 4.0) return { };

        const auto moments = cv::moments(gap_contour);
        if (moments.m00 <= 0) return { };
        const auto gap_center = cv::Point2f {
            static_cast<float>(moments.m10 / moments.m00),
            static_cast<float>(moments.m01 / moments.m00),
        };
        const auto outer_radius =
            std::max(outer_ellipse.size.width, outer_ellipse.size.height) / 2.0f;
        if (get_distance(gap_center, outer_ellipse.center) > outer_radius) return { };

        std::vector<Gap> result;
        result.reserve(4);
        for (int i = 0; i < 4; ++i) {
            const auto angle = (outer_ellipse.angle + 45.0f + static_cast<float>(i) * 90.0f)
                * std::numbers::pi / 180.0f;
            const auto direction = cv::Point2f {
                static_cast<float>(std::cos(angle)),
                static_cast<float>(std::sin(angle)),
            };
            const auto radius = static_cast<float>(input.gap_circle_radius_ratio * outer_radius);
            const auto center = outer_ellipse.center + direction * radius;

            Gap gap;
            gap.center       = center;
            gap.left_corner  = center;
            gap.right_corner = center;
            gap.valid        = true;
            result.push_back(gap);
        }

        return result;
    }

    auto build_inactive_page(const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<cv::Vec4i>& hierarchy, const std::unordered_set<size_t>& valid,
        size_t outer_index, const cv::RotatedRect& outer_ellipse, const cv::Point2f& rune_center,
        const RuneFinder::Input& input) -> std::optional<RunePage> {
        auto gaps = find_gaps(contours, hierarchy, valid, outer_index, outer_ellipse, input);

        // 如果标准缺口检测失败，尝试识别十字形缺口（小符未激活页）
        if (gaps.empty()) {
            std::vector<int> direct_children;
            get_direct_sub_contour_indices(hierarchy, outer_index, direct_children);
            for (const auto child_idx : direct_children) {
                if (child_idx < 0 || child_idx >= static_cast<int>(contours.size())) continue;
                if (!is_contour_valid(child_idx, valid)) continue;
                auto cross_gaps = make_crosshair_gaps(contours[child_idx], outer_ellipse, input);
                if (!cross_gaps.empty()) {
                    gaps = std::move(cross_gaps);
                    break;
                }
            }
        }

        if (gaps.empty() || gaps.size() > 4) return std::nullopt;

        RunePage page;
        page.active = false;
        page.center = to_point2d(outer_ellipse.center);

        const auto sorted_gaps = sort_gaps(gaps, outer_ellipse.center, rune_center);
        for (size_t index = 0; index < 4; ++index) {
            page.gap_valid[index] = sorted_gaps[index].valid;
            page.gap_corners[index] =
                sorted_gaps[index].valid ? to_point2d(sorted_gaps[index].center) : Point2d { };
        }

        return page;
    }

    auto find_inactive_pages(const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<cv::Vec4i>& hierarchy, const std::unordered_set<size_t>& valid,
        const cv::Point2f& rune_center, const RuneFinder::Input& input) -> std::vector<RunePage> {
        std::vector<RunePage> result;

        for (size_t index = 0; index < contours.size(); ++index) {
            if (!is_contour_valid(index, valid)) continue;
            if (hierarchy[index][3] != -1) continue;
            if (hierarchy[index][2] == -1) continue;

            std::vector<int> direct_children;
            get_direct_sub_contour_indices(hierarchy, index, direct_children);
            if (direct_children.empty()) continue;

            const auto maybe_ellipse = check_ellipse_shape(contours[index], input);
            if (!maybe_ellipse) continue;

            const auto maybe_page = build_inactive_page(
                contours, hierarchy, valid, index, *maybe_ellipse, rune_center, input);
            if (maybe_page) {
                result.push_back(*maybe_page);
            }
        }

        return result;
    }

} // namespace

auto RuneFinder::solve() noexcept -> bool {
    result.icon = Point2d { };
    result.pages.clear();

    if (input.image.empty()) return false;
    if (input.color == CampColor::UNKNOWN) return false;

    const auto binary_image = make_binary_image(input.image, input);
    if (binary_image.empty()) return false;

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    const auto common_min_area = std::min(input.min_area, input.center_min_area);
    const auto common_max_area = std::max(input.max_area, input.center_max_area);
    const auto valid_indices =
        get_valid_contour_indices(contours, common_min_area, common_max_area);
    if (valid_indices.empty()) return false;

    const auto maybe_center = find_center(contours, hierarchy, valid_indices, input);
    if (maybe_center) {
        result.icon = to_point2d(*maybe_center);
    }

    auto active_pages = find_active_pages(contours, hierarchy, valid_indices, maybe_center, input);
    result.pages.insert(result.pages.end(), active_pages.begin(), active_pages.end());

    if (maybe_center) {
        auto inactive_pages =
            find_inactive_pages(contours, hierarchy, valid_indices, *maybe_center, input);
        result.pages.insert(result.pages.end(), inactive_pages.begin(), inactive_pages.end());
    }

    return !result.pages.empty();
}

} // namespace rmcs

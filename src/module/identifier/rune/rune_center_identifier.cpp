#include "rune_center_identifier.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "utility/serializable.hpp"

namespace rmcs::identifier {

struct RuneCenterIdentifier::Impl {
    struct Config : util::Serializable {
        double min_area                   = 10.0;
        double max_area                   = 1500.0;
        double min_side_ratio             = 0.2;
        double max_side_ratio             = 2.5;
        double min_roundness              = 0.1;
        double max_roundness              = 0.9;
        double max_sub_area_ratio         = 0.5;
        double min_convex_area_ratio      = 0.3;
        double max_defect_area_ratio      = 0.3;
        double min_area_for_ratio         = 70.0;
        double center_concentricity_ratio = 0.08;

        constexpr static std::tuple metas {
            &Config::min_area,
            "min_area",
            &Config::max_area,
            "max_area",
            &Config::min_side_ratio,
            "min_side_ratio",
            &Config::max_side_ratio,
            "max_side_ratio",
            &Config::min_roundness,
            "min_roundness",
            &Config::max_roundness,
            "max_roundness",
            &Config::max_sub_area_ratio,
            "max_sub_area_ratio",
            &Config::min_convex_area_ratio,
            "min_convex_area_ratio",
            &Config::max_defect_area_ratio,
            "max_defect_area_ratio",
            &Config::min_area_for_ratio,
            "min_area_for_ratio",
            &Config::center_concentricity_ratio,
            "center_concentricity_ratio",
        };
    } config;

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) return std::unexpected { result.error() };

        return {};
    }

    auto identify(const RunePreprocessResult& input) const noexcept
        -> std::expected<std::vector<RuneCenterCandidate>, std::string> {
        if (input.candidates.size() != input.hierarchy.size()) {
            return std::unexpected {
                "RunePreprocessResult candidates and hierarchy size mismatch.",
            };
        }

        auto centers = std::vector<RuneCenterCandidate> {};
        for (int index = 0; index < static_cast<int>(input.candidates.size()); ++index) {
            auto center = build_center_candidate(input, index);
            if (!center.has_value()) continue;

            centers.push_back(*center);
        }

        return centers;
    }

private:
    static auto collect_sibling_chain(const std::vector<cv::Vec4i>& hierarchy, int any_index)
        -> std::vector<int> {
        if (any_index == -1) return {};

        auto first_index = any_index;
        while (hierarchy[static_cast<std::size_t>(first_index)][1] != -1) {
            first_index = hierarchy[static_cast<std::size_t>(first_index)][1];
        }

        auto siblings = std::vector<int> {};
        for (auto index = first_index; index != -1;
            index       = hierarchy[static_cast<std::size_t>(index)][0]) {
            siblings.push_back(index);
        }
        return siblings;
    }

    static auto collect_direct_children(const std::vector<cv::Vec4i>& hierarchy, int parent_index)
        -> std::vector<int> {
        if (parent_index < 0) return {};
        return collect_sibling_chain(
            hierarchy, hierarchy[static_cast<std::size_t>(parent_index)][2]);
    }

    static auto collect_all_descendants(const std::vector<cv::Vec4i>& hierarchy, int parent_index)
        -> std::vector<int> {
        auto descendants              = std::vector<int> {};
        const auto append_descendants = [&](this const auto& self, int current_index) -> void {
            for (const auto child_index : collect_direct_children(hierarchy, current_index)) {
                descendants.push_back(child_index);
                self(child_index);
            }
        };
        append_descendants(parent_index);
        return descendants;
    }

    static auto contour_center(const std::vector<cv::Point>& contour) -> cv::Point2f {
        const auto moments = cv::moments(contour);
        if (std::abs(moments.m00) < 1e-6) {
            const auto rect = cv::boundingRect(contour);
            return cv::Point2f {
                rect.x + 0.5F * static_cast<float>(rect.width),
                rect.y + 0.5F * static_cast<float>(rect.height),
            };
        }

        return cv::Point2f {
            static_cast<float>(moments.m10 / moments.m00),
            static_cast<float>(moments.m01 / moments.m00),
        };
    }

    static auto fitted_shape(const RuneContourCandidate& candidate) -> cv::RotatedRect {
        if (candidate.contour.size() >= 6) {
            return cv::fitEllipse(candidate.contour);
        }
        return candidate.rotated_rect;
    }

    auto is_hierarchy_center(const RunePreprocessResult& input, int index) const -> bool {
        const auto& hierarchy = input.hierarchy;
        const auto& candidate = input.candidates[static_cast<std::size_t>(index)];
        const auto& node      = hierarchy[static_cast<std::size_t>(index)];

        if ((node[0] == -1 && node[1] == -1) || node[3] != -1) {
            return false;
        }
        if (node[2] == -1) {
            return true;
        }

        const auto first_child = node[2];
        if (hierarchy[static_cast<std::size_t>(first_child)][2] != -1) return false;

        const auto outer_shape = fitted_shape(candidate);
        const auto inner_center =
            contour_center(input.candidates[static_cast<std::size_t>(first_child)].contour);
        const auto direction = inner_center - outer_shape.center;
        const auto distance  = std::sqrt(direction.dot(direction));
        const auto mean_size = 0.5 * (outer_shape.size.width + outer_shape.size.height);
        if (mean_size <= 0.0) return false;

        if (distance / mean_size > config.center_concentricity_ratio) return true;

        return input.candidates[static_cast<std::size_t>(first_child)].contour.size() < 10U;
    }

    auto build_center_candidate(const RunePreprocessResult& input, int index) const
        -> std::optional<RuneCenterCandidate> {
        const auto& candidate = input.candidates[static_cast<std::size_t>(index)];
        if (candidate.contour.size() < 6U) return std::nullopt;

        if (!is_hierarchy_center(input, index)) return std::nullopt;

        if (candidate.area < config.min_area || candidate.area > config.max_area)
            return std::nullopt;

        const auto shape  = fitted_shape(candidate);
        const auto width  = std::max(shape.size.width, shape.size.height);
        const auto height = std::min(shape.size.width, shape.size.height);
        if (height <= 0.0) return std::nullopt;

        const auto side_ratio = width / height;
        if (side_ratio < config.min_side_ratio || side_ratio > config.max_side_ratio)
            return std::nullopt;

        const auto perimeter = cv::arcLength(candidate.contour, true);
        if (perimeter <= 0.0) return std::nullopt;

        const auto roundness = 4.0 * std::numbers::pi * candidate.area / (perimeter * perimeter);
        if (roundness < config.min_roundness || roundness > config.max_roundness) {
            return std::nullopt;
        }

        const auto descendants = collect_all_descendants(input.hierarchy, index);
        auto total_sub_area    = 0.0;
        for (const auto descendant_index : descendants) {
            total_sub_area += input.candidates[static_cast<std::size_t>(descendant_index)].area;
        }
        if (candidate.area > config.min_area_for_ratio
            && total_sub_area / candidate.area > config.max_sub_area_ratio) {
            return std::nullopt;
        }

        auto hull = std::vector<cv::Point> {};
        cv::convexHull(candidate.contour, hull);
        const auto convex_area = cv::contourArea(hull);
        if (convex_area <= 0.0) return std::nullopt;

        if (candidate.area > config.min_area_for_ratio
            && candidate.area / convex_area < config.min_convex_area_ratio) {
            return std::nullopt;
        }

        if (candidate.area > config.min_area_for_ratio && !cv::isContourConvex(candidate.contour)) {
            auto hull_indices = std::vector<int> {};
            cv::convexHull(candidate.contour, hull_indices);
            if (hull_indices.size() >= 3U) {
                auto defects = std::vector<cv::Vec4i> {};
                cv::convexityDefects(candidate.contour, hull_indices, defects);

                auto max_defect_area = 0.0;
                for (const auto& defect : defects) {
                    const auto start       = candidate.contour[static_cast<std::size_t>(defect[0])];
                    const auto end         = candidate.contour[static_cast<std::size_t>(defect[1])];
                    const auto depth       = defect[3] / 256.0;
                    const auto base_length = cv::norm(end - start);
                    max_defect_area        = std::max(max_defect_area, 0.5 * base_length * depth);
                }

                if (max_defect_area / candidate.area > config.max_defect_area_ratio)
                    return std::nullopt;
            }
        }

        return RuneCenterCandidate {
            .contour_index = index,
            .area          = candidate.area,
            .center        = shape.center,
            .rotated_rect  = shape,
        };
    }
};

RuneCenterIdentifier::RuneCenterIdentifier() noexcept
    : pimpl(std::make_unique<Impl>()) { }

RuneCenterIdentifier::~RuneCenterIdentifier() noexcept = default;

auto RuneCenterIdentifier::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto RuneCenterIdentifier::identify(const RunePreprocessResult& input) const noexcept
    -> std::expected<std::vector<RuneCenterCandidate>, std::string> {
    return pimpl->identify(input);
}

} // namespace rmcs::identifier

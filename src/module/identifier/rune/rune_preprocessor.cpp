#include "rune_preprocessor.hpp"

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "utility/image/image.details.hpp"
#include "utility/robot/color.hpp"
#include "utility/serializable.hpp"

using namespace rmcs;
using namespace rmcs::identifier;

struct RunePreprocessor::Impl {
    struct Config : util::Serializable {
        std::string target_color;
        int binary_threshold    = 40;
        double min_contour_area = 20.0;
        double max_contour_area = 10000.0;

        constexpr static std::tuple metas {
            &Config::target_color,
            "target_color",
            &Config::binary_threshold,
            "binary_threshold",
            &Config::min_contour_area,
            "min_contour_area",
            &Config::max_contour_area,
            "max_contour_area",
        };
    } config;

    CampColor target_color { CampColor::UNKNOWN };

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
        if (parent_index != -1) {
            return collect_sibling_chain(
                hierarchy, hierarchy[static_cast<std::size_t>(parent_index)][2]);
        }

        auto roots   = std::vector<int> {};
        auto visited = std::vector<bool>(hierarchy.size(), false);
        for (int index = 0; index < static_cast<int>(hierarchy.size()); ++index) {
            if (hierarchy[static_cast<std::size_t>(index)][3] != -1
                || hierarchy[static_cast<std::size_t>(index)][1] != -1) {
                continue;
            }

            for (const auto root_index : collect_sibling_chain(hierarchy, index)) {
                if (hierarchy[static_cast<std::size_t>(root_index)][3] != -1
                    || visited[static_cast<std::size_t>(root_index)]) {
                    continue;
                }

                roots.push_back(root_index);
                visited[static_cast<std::size_t>(root_index)] = true;
            }
        }

        for (int index = 0; index < static_cast<int>(hierarchy.size()); ++index) {
            if (hierarchy[static_cast<std::size_t>(index)][3] == -1
                && !visited[static_cast<std::size_t>(index)]) {
                roots.push_back(index);
            }
        }

        return roots;
    }

    static auto collect_promoted_children(const std::vector<cv::Vec4i>& hierarchy,
        const std::vector<bool>& keep, int parent_index) -> std::vector<int> {
        // 如果中间轮廓被过滤掉，但更深层的子轮廓被保留下来，
        // 需要把这些子轮廓向上提升，保持剩余轮廓树连通。
        auto children = std::vector<int> {};
        for (const auto child_index : collect_direct_children(hierarchy, parent_index)) {
            if (keep[static_cast<std::size_t>(child_index)]) {
                children.push_back(child_index);
                continue;
            }

            auto promoted = collect_promoted_children(hierarchy, keep, child_index);
            children.insert(children.end(), promoted.begin(), promoted.end());
        }
        return children;
    }

    static auto rebuild_hierarchy(const std::vector<cv::Vec4i>& hierarchy,
        const std::vector<bool>& keep, const std::vector<int>& old_to_new, std::size_t kept_count)
        -> std::vector<cv::Vec4i> {
        // 过滤轮廓后，原始 hierarchy 的索引和拓扑都可能失效，
        // 这里基于保留轮廓重建一份新的层级关系，供后续 rune 分类直接使用。
        auto rebuilt = std::vector<cv::Vec4i>(kept_count, cv::Vec4i { -1, -1, -1, -1 });

        const auto assign_children = [&](this const auto& self, int old_parent,
                                         int new_parent) -> void {
            auto promoted_children = collect_promoted_children(hierarchy, keep, old_parent);
            for (std::size_t index = 0; index < promoted_children.size(); ++index) {
                const auto old_child  = promoted_children[index];
                const auto new_child  = old_to_new[static_cast<std::size_t>(old_child)];
                auto& child_hierarchy = rebuilt[static_cast<std::size_t>(new_child)];

                child_hierarchy[0] = index + 1 < promoted_children.size()
                    ? old_to_new[static_cast<std::size_t>(promoted_children[index + 1])]
                    : -1;
                child_hierarchy[1] = index > 0
                    ? old_to_new[static_cast<std::size_t>(promoted_children[index - 1])]
                    : -1;
                child_hierarchy[3] = new_parent;

                if (new_parent != -1 && index == 0) {
                    rebuilt[static_cast<std::size_t>(new_parent)][2] = new_child;
                }

                self(old_child, new_child);
            }
        };

        assign_children(-1, -1);
        return rebuilt;
    }

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        // 在初始化阶段一次性解析目标颜色，避免每帧重复决定通道差分方向
        if (config.target_color == "red") {
            target_color = CampColor::RED;
        } else if (config.target_color == "blue") {
            target_color = CampColor::BLUE;
        } else {
            return std::unexpected { "target_color should be [red] or [blue]." };
        }

        if (config.binary_threshold < 0 || config.binary_threshold > 255) {
            return std::unexpected { "binary_threshold should be in [0, 255]." };
        }
        if (config.min_contour_area < 0.0) {
            return std::unexpected { "min_contour_area should be non-negative." };
        }
        if (config.max_contour_area < config.min_contour_area) {
            return std::unexpected {
                "max_contour_area should be greater than or equal to min_contour_area.",
            };
        }

        return {};
    }

    auto sync_process(const Image& image) const noexcept -> std::optional<RunePreprocessResult> {
        const auto& source = image.details().mat;
        if (source.empty()) {
            return std::nullopt;
        }
        if (source.type() != CV_8UC3) {
            return std::nullopt;
        }
        if (target_color != CampColor::RED && target_color != CampColor::BLUE) {
            return std::nullopt;
        }

        cv::Mat binary              = cv::Mat::zeros(source.size(), CV_8UC1);
        const auto positive_channel = target_color == CampColor::RED ? 2 : 0;
        const auto negative_channel = target_color == CampColor::RED ? 0 : 2;

        // 根据配置的目标颜色构造颜色差分二值图,红色目标使用 R-B，蓝色目标使用 B-R
        cv::parallel_for_(cv::Range(0, source.rows), [&](const cv::Range& range) {
            for (int row = range.start; row < range.end; ++row) {
                const auto* source_ptr = source.ptr<cv::Vec3b>(row);
                auto* binary_ptr       = binary.ptr<std::uint8_t>(row);

                for (int col = 0; col < source.cols; ++col) {
                    const auto diff = static_cast<int>(source_ptr[col][positive_channel])
                        - static_cast<int>(source_ptr[col][negative_channel]);
                    if (diff > config.binary_threshold) {
                        binary_ptr[col] = 255;
                    }
                }
            }
        });

        auto contours  = std::vector<std::vector<cv::Point>> {};
        auto hierarchy = std::vector<cv::Vec4i> {};
        // 保留完整轮廓树结构,后续 rune 特征分类会依赖轮廓的父子层级关系
        cv::findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

        auto keep              = std::vector<bool>(contours.size(), false);
        auto old_to_new        = std::vector<int>(contours.size(), -1);
        auto filtered_contours = std::vector<std::vector<cv::Point>> {};
        filtered_contours.reserve(contours.size());

        // 先按面积筛掉明显无效的轮廓，再基于保留下来的轮廓重建 hierarchy，
        // 保证返回给下游的层级索引始终有效
        for (std::size_t index = 0; index < contours.size(); ++index) {
            const auto area = cv::contourArea(contours[index]);
            if (area < config.min_contour_area || area > config.max_contour_area) {
                continue;
            }

            keep[index]       = true;
            old_to_new[index] = static_cast<int>(filtered_contours.size());
            filtered_contours.push_back(contours[index]);
        }

        auto result      = RunePreprocessResult {};
        result.hierarchy = rebuild_hierarchy(hierarchy, keep, old_to_new, filtered_contours.size());
        result.candidates.reserve(filtered_contours.size());

        // 预先缓存后续 rune 分类会频繁使用的几何信息，避免下游重复计算
        for (const auto& contour : filtered_contours) {
            const auto rotated_rect = cv::minAreaRect(contour);
            result.candidates.push_back(RuneContourCandidate {
                .contour       = contour,
                .area          = cv::contourArea(contour),
                .bounding_rect = cv::boundingRect(contour),
                .rotated_rect  = rotated_rect,
                .center        = rotated_rect.center,
            });
        }

        return result;
    }
};

RunePreprocessor::RunePreprocessor() noexcept
    : pimpl(std::make_unique<Impl>()) { }

RunePreprocessor::~RunePreprocessor() noexcept = default;

auto RunePreprocessor::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto RunePreprocessor::sync_process(const Image& image) noexcept
    -> std::optional<RunePreprocessResult> {
    return pimpl->sync_process(image);
}

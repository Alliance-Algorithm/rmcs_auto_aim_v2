#include "pose_estimator.hpp"
#include "module/identifier/adjacency_lightbar.hpp"

#include "utility/math/conversion.hpp"
#include "utility/math/outpost.hpp"
#include "utility/math/solve_pnp/outpost_distance_optimizer.hpp"
#include "utility/math/solve_pnp/pnp_solution.hpp"
#include "utility/math/solve_pnp/yaw_optimizer.hpp"
#include "utility/serializable.hpp"

#include <algorithm>
#include <iterator>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace rmcs::util;
using namespace rmcs;

namespace rmcs::kernel {

struct PoseEstimator::Impl {
    struct Config : util::Serializable {
        bool yaw_optimizer;
        bool distance_optimizer;

        double outpost_armor_thickness;

        constexpr static std::tuple metas {
            // clang-format off
            &Config::yaw_optimizer, "yaw_optimizer",
            &Config::distance_optimizer, "distance_optimizer",

            &Config::outpost_armor_thickness, "outpost_armor_thickness"
            // clang-format on
        };
    };
    Config config;
    Addition addition { };

    CameraFeature camera_feature;

    PnpSolution pnp_solution { };
    OutpostDistanceOptimizer outpost_distance_optimizer { };
    YawOptimizer yaw_optimizer { };
    AdjacencyLightbarFinder adjacency_finder { };

    Eigen::Vector3d camera_translation { Eigen::Vector3d::Zero() };
    Eigen::Quaterniond camera_orientation { Eigen::Quaterniond::Identity() };

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> try {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        outpost_distance_optimizer.input.armor_thickness = config.outpost_armor_thickness;
        adjacency_finder.set_armor_thickness(config.outpost_armor_thickness);

        return { };
    } catch (const std::exception& e) {
        return std::unexpected { e.what() };
    }

    auto configure_camera(std::array<double, 9> camera_matrix, std::array<double, 5> distort_coeff)
        -> void {
        camera_feature.from(camera_matrix);
        camera_feature.from(distort_coeff);

        pnp_solution.input.camera               = camera_feature;
        yaw_optimizer.input.camera              = camera_feature;
        outpost_distance_optimizer.input.camera = camera_feature;
        adjacency_finder.set_camera_feature(camera_feature);
    }

    auto into_odom_link(const Translation& translation) {
        auto result = translation;

        result = Eigen::Vector3d { camera_orientation * result.make<Eigen::Vector3d>()
            + camera_translation };

        return result;
    }

    auto into_odom_link(const Armor3d& armor) const {
        auto transformed = armor;

        auto position = Eigen::Vector3d { };
        transformed.translation.copy_to(position);
        transformed.translation = camera_orientation * position + camera_translation;

        auto quat = Eigen::Quaterniond { };
        transformed.orientation.copy_to(quat);
        transformed.orientation = camera_orientation * quat;

        return transformed;
    }
    auto into_odom_link(const Lightbar3d& lightbar) const {
        auto result = lightbar;

        result.upper = Eigen::Vector3d {
            camera_orientation * result.upper.make<Eigen::Vector3d>() + camera_translation,
        };
        result.lower = Eigen::Vector3d {
            camera_orientation * result.lower.make<Eigen::Vector3d>() + camera_translation,
        };

        return result;
    }

    auto into_odom_link(std::span<const Armor3d> armors) const {
        auto result = std::vector<Armor3d> { };
        for (const auto& armor : armors) {
            result.emplace_back(into_odom_link(armor));
        }
        return result;
    }

    auto into_camera_link(const Armor3d& armor) const {
        auto transformed = armor;

        auto position = Eigen::Vector3d { };
        transformed.translation.copy_to(position);
        transformed.translation = camera_orientation.inverse() * (position - camera_translation);

        auto quat = Eigen::Quaterniond { };
        transformed.orientation.copy_to(quat);
        transformed.orientation = camera_orientation.inverse() * quat;

        return transformed;
    }

    auto solve_armor(const std::vector<Armor2d>& armors) {

        const auto q_camera_to_odom = camera_orientation;
        const auto q_odom_to_camera = q_camera_to_odom.inverse();
        const auto center_yaw       = eulers(q_camera_to_odom, 2, 1, 0)[0];

        auto& input = yaw_optimizer.input;

        input.camera.orientation = q_odom_to_camera;
        input.camera.translation =
            Eigen::Vector3d { -(q_odom_to_camera * camera_translation).eval() };

        auto result = std::vector<Armor3d> { };
        for (auto&& [index, armor] : armors | std::views::enumerate) {

            const auto small = armor.shape == ArmorShape::SMALL;
            const auto shape = small ? kSmallArmorShapeOpenCV : kLargeArmorShapeOpenCV;

            auto armor_3d = Armor3d { };
            armor_3d.id   = static_cast<int>(index);

            { // pnp
                auto& input       = pnp_solution.input;
                input.armor_shape = shape;
                input.genre       = armor.genre;
                input.color       = armor_color2camp_color(armor.color);
                std::ranges::copy(armor.corners(), input.armor_detection.begin());

                auto solved = pnp_solution.solve();
                if (!solved) continue;

                armor_3d.genre       = pnp_solution.result.genre;
                armor_3d.color       = camp_color2armor_color(pnp_solution.result.color);
                armor_3d.translation = pnp_solution.result.translation;
                armor_3d.orientation = pnp_solution.result.orientation;
            }
            if (config.yaw_optimizer) { // yaw
                input.armor_shape  = shape;
                input.xyz_in_world = armor_3d.translation;
                input.center_yaw   = center_yaw;
                input.genre        = armor.genre;
                std::ranges::copy(armor.corners(), input.detected_corners.begin());

                armor_3d.orientation = yaw_optimizer.solve().orientation;
            }
            /// @FIXME:
            ///  确认一下yaw_optimizer 内部的坐标系变换
            armor_3d = into_odom_link(armor_3d);

            result.emplace_back(armor_3d);
        };
        return result;
    }
    auto solve_armor(const std::vector<Armor2d>& armor2ds, Image& image) {
        auto armor3ds = solve_armor(armor2ds);

        if (!config.distance_optimizer) return armor3ds;
        {
            addition.areas.clear();
            addition.origin.clear();
            addition.predicted_near.clear();
            addition.predicted_away.clear();
            addition.detected_2d.clear();
            addition.detected_3d.clear();
            addition.center = { };
        }

        // 前哨站专属优化
        auto outpost2d = std::ranges::find_if(
            armor2ds, [](const Armor2d& armor) { return armor.genre == DeviceId::OUTPOST; });
        auto outpost3d = std::ranges::find_if(
            armor3ds, [](const Armor3d& armor) { return armor.genre == DeviceId::OUTPOST; });

        if (outpost3d == armor3ds.end() || outpost2d == armor2ds.end()) {
            return armor3ds;
        }

        auto outpost_in_camera = into_camera_link(*outpost3d);
        if (auto result = adjacency_finder.find(image, *outpost2d, outpost_in_camera)) {
            { // 更新附加信息，供外部绘制或者发布调试 Topic
                std::ranges::copy(result->areas, std::back_inserter(addition.areas));
                std::ranges::copy(
                    result->predicted_near, std::back_inserter(addition.predicted_near));
                std::ranges::copy(
                    result->predicted_away, std::back_inserter(addition.predicted_away));
                std::ranges::copy(result->found, std::back_inserter(addition.detected_2d));
                addition.center = result->center;
            }

            if (!result->found.empty()) {
                const auto& lightbar = result->found[0];

                auto& input   = outpost_distance_optimizer.input;
                input.initial = outpost_in_camera;

                input.armor       = *outpost2d;
                input.upper_point = lightbar.upper;
                input.lower_point = lightbar.lower;

                input.is_right = lightbar.is_right;
                input.is_upper = lightbar.is_upper;

                if (outpost_distance_optimizer.solve()) {
                    addition.origin.push_back(*outpost3d);

                    auto& result = outpost_distance_optimizer.result;

                    outpost_in_camera = result.armor;
                    *outpost3d        = into_odom_link(result.armor);

                    // 投影到 2d 看效果
                    auto solution                  = NeighborBarSolution { };
                    solution.input.source          = outpost_in_camera;
                    solution.input.in_right        = lightbar.is_right;
                    solution.input.armor_thickness = config.outpost_armor_thickness;
                    solution.solve();

                    const auto& near_bar =
                        lightbar.is_upper ? solution.result.upper_near : solution.result.lower_near;
                    const auto& away_bar =
                        lightbar.is_upper ? solution.result.upper_away : solution.result.lower_away;

                    addition.detected_3d.push_back(into_odom_link(near_bar));
                    addition.detected_3d.push_back(into_odom_link(away_bar));

                    const auto color      = ArmorVisualColor { lightbar.color };
                    const auto draw_color = std::array {
                        cv::Scalar { color.b() * 255, color.g() * 255, color.r() * 255 },
                        cv::Scalar { color.b() * 127.5, color.g() * 127.5, color.r() * 127.5 },
                    };
                    const auto bars = std::array { near_bar, away_bar };
                    for (std::size_t index = 0; index < bars.size(); ++index) {
                        const auto& bar     = bars[index];
                        auto segment_points = std::array<cv::Point3f, 2> { };
                        for (std::size_t i = 0; i < segment_points.size(); ++i) {
                            const auto point  = i == 0 ? bar.upper : bar.lower;
                            const auto p      = ros2opencv_position(point.make<Eigen::Vector3d>());
                            segment_points[i] = cv::Point3f(static_cast<float>(p[0]),
                                static_cast<float>(p[1]), static_cast<float>(p[2]));
                        }

                        auto projected = std::vector<cv::Point2f> { };
                        cv::projectPoints(segment_points, cv::Vec3d { 0.0, 0.0, 0.0 },
                            cv::Vec3d { 0.0, 0.0, 0.0 }, camera_feature.intrinsic(),
                            camera_feature.distortion(), projected);
                        if (projected.size() != 2) continue;

                        addition.detected_2d.push_back(Lightbar2d {
                            .color      = bar.color,
                            .upper      = Point2d { projected[0] },
                            .lower      = Point2d { projected[1] },
                            .draw_color = draw_color[index],
                        });
                    }
                }
            }
        }
        return armor3ds;
    }

    auto update_camera_transform(const Transform& transform) {
        transform.translation.copy_to(camera_translation);
        transform.orientation.copy_to(camera_orientation);

        const auto q_odom_to_camera = camera_orientation.inverse();
        camera_feature.orientation  = Orientation { q_odom_to_camera };
        camera_feature.translation =
            Translation { -(q_odom_to_camera * camera_translation).eval() };

        adjacency_finder.set_camera_feature(camera_feature);
    }
};

auto PoseEstimator::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto PoseEstimator::configure_camera(
    std::array<double, 9> camera_matrix, std::array<double, 5> distort_coeff) -> void {
    return pimpl->configure_camera(camera_matrix, distort_coeff);
}

auto PoseEstimator::estimate_armor(const std::vector<Armor2d>& armors) const -> Armor3ds {
    return pimpl->solve_armor(armors);
}

auto PoseEstimator::estimate_armor(const std::vector<Armor2d>& armors, Image& image) const
    -> Armor3ds {
    return pimpl->solve_armor(armors, image);
}

auto PoseEstimator::addition() -> const Addition& { return pimpl->addition; }

auto PoseEstimator::update_camera_transform(const Transform& transform) -> void {
    return pimpl->update_camera_transform(transform);
}

auto PoseEstimator::into_odom_link(std::span<const Armor3d> armors) const -> Armor3ds {
    return pimpl->into_odom_link(armors);
}
auto PoseEstimator::into_odom_link(const Armor3d& armor) const -> Armor3d {
    return pimpl->into_odom_link(armor);
}

PoseEstimator::PoseEstimator() noexcept
    : pimpl { std::make_unique<Impl>() } { }

PoseEstimator::~PoseEstimator() noexcept = default;
}

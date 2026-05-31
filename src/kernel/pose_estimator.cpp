#include "pose_estimator.hpp"
#include "module/identifier/adjacency_lightbar.hpp"

#include "utility/image/image.details.hpp"
#include "utility/math/conversion.hpp"
#include "utility/math/outpost.hpp"
#include "utility/math/solve_pnp/outpost_distance_optimizer.hpp"
#include "utility/math/solve_pnp/pnp_solution.hpp"
#include "utility/math/solve_pnp/solve_pnp.hpp"
#include "utility/math/solve_pnp/yaw_optimizer.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/visual/armor.hpp"
#include "utility/rclcpp/visual/lightbar.hpp"
#include "utility/serializable.hpp"

#include <opencv2/imgproc.hpp>

using namespace rmcs::util;
using namespace rmcs;

namespace rmcs::kernel {

namespace { } // namespace

struct PoseEstimator::Impl {
    static constexpr auto kCameraLink = "camera_link";

    struct DebugState {
        std::optional<Armor3D> pre_optimized_outpost;
        std::optional<Armor3D> optimized_outpost;
        bool has_detected_lightbar { false };
        bool has_candidate_roi { false };
        bool optimized_bar_is_right { false };
        bool optimized_bar_is_upper { false };

        auto clear() -> void { *this = { }; }
    };

    struct Config : util::Serializable {
        std::array<float, 9> camera_matrix;
        std::array<float, 5> distort_coeff;
        bool yaw_optimizer;
        bool distance_optimizer;

        constexpr static std::tuple metas { //
            &Config::camera_matrix, "camera_matrix", &Config::distort_coeff, "distort_coeff",
            &Config::yaw_optimizer, "yaw_optimizer", &Config::distance_optimizer,
            "distance_optimizer"
        };
    };

    DebugState debug_state { };

    Config config;
    CameraFeature camera_feature;

    PnpSolution pnp_solution { };
    OutpostDistanceOptimizer outpost_distance_optimizer { };
    YawOptimizer yaw_optimizer { };
    AdjacencyLightbarFinder adjacency_finder { };

    RclcppNode visual_node { "PoseEstimatorVisual" };
    std::unique_ptr<visual::Armor> pre_optimized_outpost;
    std::unique_ptr<visual::LightBar> optimized_neighbor_bar_near;
    std::unique_ptr<visual::LightBar> optimized_neighbor_bar_away;

    Eigen::Vector3d camera_translation { Eigen::Vector3d::Zero() };
    Eigen::Quaterniond camera_orientation { Eigen::Quaterniond::Identity() };

    Impl() noexcept { visual_node.set_pub_topic_prefix("/rmcs/auto_aim/"); }

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> try {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        camera_feature.camera_matrix = reshape_array<float, 9, double, 3, 3>(config.camera_matrix);
        camera_feature.distort_coeff = reshape_array<float, 5, double>(config.distort_coeff);

        pnp_solution.input.camera               = camera_feature;
        yaw_optimizer.input.camera              = camera_feature;
        outpost_distance_optimizer.input.camera = camera_feature;
        adjacency_finder.set_camera_feature(camera_feature);

        return { };
    } catch (const std::exception& e) {
        return std::unexpected { e.what() };
    }

    auto solve_armor(const std::vector<Armor2D>& armors) {

        const auto q_camera_to_odom = camera_orientation;
        const auto q_odom_to_camera = q_camera_to_odom.inverse();
        const auto center_yaw       = eulers(q_camera_to_odom, 2, 1, 0)[0];

        auto& input = yaw_optimizer.input;

        input.camera.camera_orientation = q_odom_to_camera;
        input.camera.camera_translation =
            Eigen::Vector3d { -(q_odom_to_camera * camera_translation).eval() };

        auto result = std::vector<Armor3D> { };
        for (auto&& [index, armor] : armors | std::views::enumerate) {

            const auto small = armor.shape == ArmorShape::SMALL;
            const auto shape = small ? kSmallArmorShapeOpenCV : kLargeArmorShapeOpenCV;

            auto armor_3d = Armor3D { };
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
                auto t_in_camera = pnp_solution.result.translation.make<Eigen::Vector3d>();
                auto t_in_world  = Eigen::Vector3d {
                    (q_camera_to_odom * t_in_camera + camera_translation).eval()
                };

                input.armor_shape  = shape;
                input.xyz_in_world = Translation { t_in_world };
                input.center_yaw   = center_yaw;
                input.genre        = armor.genre;
                std::ranges::copy(armor.corners(), input.detected_corners.begin());

                armor_3d.orientation = yaw_optimizer.solve().orientation;
            }

            result.emplace_back(armor_3d);
        };
        return result;
    }
    auto solve_armor(const std::vector<Armor2D>& armor2ds, Image& image) {
        auto armor3ds = solve_armor(armor2ds);
        debug_state.clear();

        // 前哨站专属优化
        auto outpost2d = std::ranges::find_if(
            armor2ds, [](const Armor2D& armor) { return armor.genre == DeviceId::OUTPOST; });
        auto outpost3d = std::ranges::find_if(
            armor3ds, [](const Armor3D& armor) { return armor.genre == DeviceId::OUTPOST; });
        if (outpost3d != armor3ds.end() && outpost2d != armor2ds.end()) {
            debug_state.pre_optimized_outpost = *outpost3d;

            if (auto lightbar = adjacency_finder.find(image, *outpost2d, *outpost3d)) {
                debug_state.has_detected_lightbar = true;
                debug_state.has_candidate_roi     = true;
                if (config.distance_optimizer) {
                    auto& input   = outpost_distance_optimizer.input;
                    input.initial = *outpost3d;

                    input.armor  = *outpost2d;
                    input.point1 = lightbar->point1;
                    input.point2 = lightbar->point2;

                    input.is_right = lightbar->is_right;
                    input.is_upper = lightbar->is_upper;

                    if (outpost_distance_optimizer.solve()) {
                        *outpost3d = outpost_distance_optimizer.result.armor;

                        debug_state.optimized_outpost      = *outpost3d;
                        debug_state.optimized_bar_is_right = lightbar->is_right;
                        debug_state.optimized_bar_is_upper = lightbar->is_upper;
                    }
                }
            } else {
                debug_state.has_candidate_roi = true;
            }
        }
        return armor3ds;
    }

    auto update_camera_transform(const Transform& transform) {
        transform.translation.copy_to(camera_translation);
        transform.orientation.copy_to(camera_orientation);

        const auto q_odom_to_camera       = camera_orientation.inverse();
        camera_feature.camera_orientation = Orientation { q_odom_to_camera };
        camera_feature.camera_translation =
            Translation { -(q_odom_to_camera * camera_translation).eval() };

        adjacency_finder.set_camera_feature(camera_feature);
    }

    auto into_odom_link(const Armor3D& armor) const -> Armor3D {
        auto transformed = armor;

        auto position = Eigen::Vector3d { };
        transformed.translation.copy_to(position);
        transformed.translation = camera_orientation * position + camera_translation;

        auto quat = Eigen::Quaterniond { };
        transformed.orientation.copy_to(quat);
        transformed.orientation = camera_orientation * quat;

        return transformed;
    }

    auto into_odom_link(std::span<const Armor3D> armors) const {
        auto result = std::vector<Armor3D> { };
        for (const auto& armor : armors) {
            result.emplace_back(into_odom_link(armor));
        }
        return result;
    }

    auto draw_optimized_outpost_neighbor_bar(
        Image& image, const Armor3D& armor, bool is_right, bool is_upper) const -> void {
        auto solution           = NeighborBarSolution { };
        solution.input.source   = armor;
        solution.input.in_right = is_right;
        solution.solve();

        const auto& selected_bar =
            is_upper ? solution.result.upper_near : solution.result.lower_near;

        auto object_points = std::vector<cv::Point3f> { };
        object_points.reserve(2);
        for (const auto& point : { selected_bar.first, selected_bar.second }) {
            const auto p = ros2opencv_position(point.make<Eigen::Vector3d>());
            object_points.emplace_back(
                static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
        }

        auto projected = std::vector<cv::Point2f> { };
        cv::projectPoints(object_points, cv::Vec3d { 0.0, 0.0, 0.0 }, cv::Vec3d { 0.0, 0.0, 0.0 },
            camera_feature.intrinsic(), camera_feature.distortion(), projected);
        if (projected.size() != 2) return;

        auto& mat = image.details().mat;
        for (const auto& point : projected) {
            cv::circle(mat, point, 2, cv::Scalar { 0, 255, 255 }, -1, cv::LINE_AA);
        }
    }

    auto publish_optimized_outpost_neighbor_bar(const Armor3D& armor, bool is_right, bool is_upper)
        -> void {
        auto solution           = NeighborBarSolution { };
        solution.input.source   = armor;
        solution.input.in_right = is_right;
        solution.solve();

        const auto& near_bar = is_upper ? solution.result.upper_near : solution.result.lower_near;
        const auto& away_bar = is_upper ? solution.result.upper_away : solution.result.lower_away;

        const auto color = ArmorVisualColor { armor_color2camp_color(armor.color) };
        const auto r     = static_cast<float>(color.x);
        const auto g     = static_cast<float>(color.y);
        const auto b     = static_cast<float>(color.z);

        if (!optimized_neighbor_bar_near) {
            optimized_neighbor_bar_near =
                std::make_unique<visual::LightBar>(visual::LightBar::Config {
                    .rclcpp = visual_node,
                    .id     = 0,
                    .name   = "optimized_neighbor_bar_near",
                    .tf     = kCameraLink,
                    .r      = r,
                    .g      = g,
                    .b      = b,
                    .a      = 1.0F,
                    .width  = 0.01,
                });
        }
        if (!optimized_neighbor_bar_away) {
            optimized_neighbor_bar_away =
                std::make_unique<visual::LightBar>(visual::LightBar::Config {
                    .rclcpp = visual_node,
                    .id     = 1,
                    .name   = "optimized_neighbor_bar_away",
                    .tf     = kCameraLink,
                    .r      = r,
                    .g      = g,
                    .b      = b,
                    .a      = 1.0F,
                    .width  = 0.01,
                });
        }

        optimized_neighbor_bar_near->set(near_bar.first, near_bar.second);
        optimized_neighbor_bar_near->update();

        optimized_neighbor_bar_away->set(away_bar.first, away_bar.second);
        optimized_neighbor_bar_away->update();
    }

    auto draw_debug(Image& image) -> void {
        if (debug_state.optimized_outpost.has_value()) {
            draw_optimized_outpost_neighbor_bar(image, *debug_state.optimized_outpost,
                debug_state.optimized_bar_is_right, debug_state.optimized_bar_is_upper);
        }
        if (debug_state.has_detected_lightbar) {
            adjacency_finder.draw_lightbar(image);
        }
        if (debug_state.has_candidate_roi) {
            adjacency_finder.draw_roi(image);
        }
    }

    auto publish_debug() -> void {
        if (debug_state.pre_optimized_outpost.has_value()) {
            publish_pre_optimized_outpost(*debug_state.pre_optimized_outpost);
        }
        if (debug_state.optimized_outpost.has_value()) {
            publish_optimized_outpost_neighbor_bar(*debug_state.optimized_outpost,
                debug_state.optimized_bar_is_right, debug_state.optimized_bar_is_upper);
        }
    }

    auto publish_pre_optimized_outpost(const Armor3D& armor) -> void {
        const auto camp = armor_color2camp_color(armor.color);
        if (!pre_optimized_outpost) {
            pre_optimized_outpost = std::make_unique<visual::Armor>(visual::Armor::Config {
                .rclcpp = visual_node,
                .device = armor.genre,
                .camp   = camp,
                .id     = 0,
                .name   = "pre_optimized_outpost",
                .tf     = kCameraLink,
            });
        }
        pre_optimized_outpost->move(armor.translation, armor.orientation);
        pre_optimized_outpost->update();
    }
};

auto PoseEstimator::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto PoseEstimator::estimate_armor(const std::vector<Armor2D>& armors) const
    -> std::vector<Armor3D> {
    return pimpl->solve_armor(armors);
}

auto PoseEstimator::estimate_armor(const std::vector<Armor2D>& armors, Image& image) const
    -> std::vector<Armor3D> {
    return pimpl->solve_armor(armors, image);
}

auto PoseEstimator::update_camera_transform(const Transform& transform) -> void {
    return pimpl->update_camera_transform(transform);
}

auto PoseEstimator::into_odom_link(std::span<const Armor3D> armors) const -> std::vector<Armor3D> {
    return pimpl->into_odom_link(armors);
}
auto PoseEstimator::into_odom_link(const Armor3D& armor) const -> Armor3D {
    return pimpl->into_odom_link(armor);
}

auto PoseEstimator::draw_debug(Image& image) -> void { return pimpl->draw_debug(image); }

auto PoseEstimator::publish_debug() -> void { return pimpl->publish_debug(); }

PoseEstimator::PoseEstimator() noexcept
    : pimpl { std::make_unique<Impl>() } { }

PoseEstimator::~PoseEstimator() noexcept = default;
}

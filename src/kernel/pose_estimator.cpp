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

        auto clear() -> void { *this = {}; }
    };

    struct Config : util::Serializable {
        std::array<float, 9> camera_matrix;
        std::array<float, 5> distort_coeff;
        bool yaw_optimizer;
        bool distance_optimizer;
        double outpost_armor_thickness;

        constexpr static std::tuple metas { //
            &Config::camera_matrix, "camera_matrix", &Config::distort_coeff, "distort_coeff",
            &Config::yaw_optimizer, "yaw_optimizer", &Config::distance_optimizer,
            "distance_optimizer", &Config::outpost_armor_thickness, "outpost_armor_thickness"
        };
    };

    DebugState debug_state {};

    Config config;
    CameraFeature camera_feature;

    PnpSolution pnp_solution {};
    OutpostDistanceOptimizer outpost_distance_optimizer {};
    YawOptimizer yaw_optimizer {};
    AdjacencyLightbarFinder adjacency_finder {};

    RclcppNode visual_node { "PoseEstimatorVisual" };
    std::unique_ptr<visual::Armor> pre_optimized_outpost;
    std::unique_ptr<visual::LightBar> optimized_neighbor_bar_near;
    std::unique_ptr<visual::LightBar> optimized_neighbor_bar_away;

    Eigen::Vector3d camera_translation { Eigen::Vector3d::Zero() };
    Eigen::Quaterniond camera_orientation { Eigen::Quaterniond::Identity() };

    Impl() noexcept {
        using namespace visual;

        visual_node.set_pub_topic_prefix("/rmcs/auto_aim/");

        optimized_neighbor_bar_near = std::make_unique<LightBar>(LightBar::Config {
            .rclcpp = visual_node,
            .id     = 0,
            .name   = "optimized_neighbor_bar_near",
            .tf     = kCameraLink,
            .width  = 0.01,
        });

        optimized_neighbor_bar_away = std::make_unique<LightBar>(LightBar::Config {
            .rclcpp = visual_node,
            .id     = 1,
            .name   = "optimized_neighbor_bar_away",
            .tf     = kCameraLink,
            .width  = 0.01,
        });

        pre_optimized_outpost = std::make_unique<Armor>(Armor::Config {
            .rclcpp = visual_node,
            .device = DeviceId::OUTPOST,
            .camp   = CampColor::BLUE,
            .id     = 0,
            .name   = "pre_optimized_outpost",
            .tf     = kCameraLink,
        });
    }

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

        outpost_distance_optimizer.input.armor_thickness = config.outpost_armor_thickness;
        adjacency_finder.set_armor_thickness(config.outpost_armor_thickness);

        return {};
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

        auto result = std::vector<Armor3D> {};
        for (auto&& [index, armor] : armors | std::views::enumerate) {

            const auto small = armor.shape == ArmorShape::SMALL;
            const auto shape = small ? kSmallArmorShapeOpenCV : kLargeArmorShapeOpenCV;

            auto armor_3d = Armor3D {};
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
            armor_3d = into_odom_link(armor_3d);

            if (config.yaw_optimizer) { // yaw
                input.armor_shape  = shape;
                input.xyz_in_world = armor_3d.translation;
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
            auto camera_frame_outpost = into_camera_link(*outpost3d);

            debug_state.pre_optimized_outpost = camera_frame_outpost;

            if (auto lightbar = adjacency_finder.find(image, *outpost2d, camera_frame_outpost)) {
                if (config.distance_optimizer) {
                    auto& input   = outpost_distance_optimizer.input;
                    input.initial = camera_frame_outpost;

                    input.armor       = *outpost2d;
                    input.upper_point = lightbar->upper;
                    input.lower_point = lightbar->lower;

                    input.is_right = lightbar->is_right;
                    input.is_upper = lightbar->is_upper;

                    if (outpost_distance_optimizer.solve()) {
                        debug_state.optimized_outpost = outpost_distance_optimizer.result.armor;
                        debug_state.optimized_bar_is_right = lightbar->is_right;
                        debug_state.optimized_bar_is_upper = lightbar->is_upper;

                        *outpost3d = into_odom_link(outpost_distance_optimizer.result.armor);
                    }
                }

                debug_state.has_detected_lightbar = true;
                debug_state.has_candidate_roi     = true;
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

        auto position = Eigen::Vector3d {};
        transformed.translation.copy_to(position);
        transformed.translation = camera_orientation * position + camera_translation;

        auto quat = Eigen::Quaterniond {};
        transformed.orientation.copy_to(quat);
        transformed.orientation = camera_orientation * quat;

        return transformed;
    }

    auto into_odom_link(std::span<const Armor3D> armors) const {
        auto result = std::vector<Armor3D> {};
        for (const auto& armor : armors) {
            result.emplace_back(into_odom_link(armor));
        }
        return result;
    }

    auto into_camera_link(const Armor3D& armor) const -> Armor3D {
        auto transformed = armor;

        auto position = Eigen::Vector3d {};
        transformed.translation.copy_to(position);
        transformed.translation = camera_orientation.inverse() * (position - camera_translation);

        auto quat = Eigen::Quaterniond {};
        transformed.orientation.copy_to(quat);
        transformed.orientation = camera_orientation.inverse() * quat;

        return transformed;
    }

    auto draw_debug(Image& image) -> void {
        if (debug_state.optimized_outpost.has_value()) {
            const auto& armor = *debug_state.optimized_outpost;

            auto solution                  = NeighborBarSolution {};
            solution.input.source          = armor;
            solution.input.in_right        = debug_state.optimized_bar_is_right;
            solution.input.armor_thickness = config.outpost_armor_thickness;
            solution.solve();

            const auto& near_bar = debug_state.optimized_bar_is_upper ? solution.result.upper_near
                                                                      : solution.result.lower_near;
            const auto& away_bar = debug_state.optimized_bar_is_upper ? solution.result.upper_away
                                                                      : solution.result.lower_away;

            auto object_points = std::vector<cv::Point3f> {};
            object_points.reserve(4);
            for (const auto& point :
                { near_bar.first, near_bar.second, away_bar.first, away_bar.second }) {
                const auto p = ros2opencv_position(point.make<Eigen::Vector3d>());
                object_points.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()),
                    static_cast<float>(p.z()));
            }

            auto projected = std::vector<cv::Point2f> {};
            cv::projectPoints(object_points, cv::Vec3d { 0.0, 0.0, 0.0 },
                cv::Vec3d { 0.0, 0.0, 0.0 }, camera_feature.intrinsic(),
                camera_feature.distortion(), projected);
            if (projected.size() == 4) {
                const auto color       = ArmorVisualColor { armor.color };
                const auto near_scalar = cv::Scalar { color.z * 255, color.y * 255, color.x * 255 };
                const auto away_scalar = near_scalar * 0.5;

                auto& mat = image.details().mat;
                cv::circle(mat, projected[0], 2, near_scalar, -1, cv::LINE_AA);
                cv::circle(mat, projected[1], 2, near_scalar, -1, cv::LINE_AA);
                cv::circle(mat, projected[2], 2, away_scalar, -1, cv::LINE_AA);
                cv::circle(mat, projected[3], 2, away_scalar, -1, cv::LINE_AA);
            }
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
            const auto& armor = *debug_state.pre_optimized_outpost;
            pre_optimized_outpost->set_camp(armor_color2camp_color(armor.color));
            pre_optimized_outpost->move(armor.translation, armor.orientation);
            pre_optimized_outpost->update();
        }
        if (debug_state.optimized_outpost.has_value()) {
            const auto& armor = *debug_state.optimized_outpost;

            auto solution                  = NeighborBarSolution {};
            solution.input.source          = armor;
            solution.input.in_right        = debug_state.optimized_bar_is_right;
            solution.input.armor_thickness = config.outpost_armor_thickness;
            solution.solve();

            const auto color    = ArmorVisualColor { armor.color };
            const auto is_upper = debug_state.optimized_bar_is_upper;

            const auto& near_bar =
                is_upper ? solution.result.upper_near : solution.result.lower_near;

            optimized_neighbor_bar_near->set_color(color);
            optimized_neighbor_bar_near->set_point(near_bar.first, near_bar.second);
            optimized_neighbor_bar_near->update();

            const auto& away_bar =
                is_upper ? solution.result.upper_away : solution.result.lower_away;

            optimized_neighbor_bar_away->set_color(color * 0.5);
            optimized_neighbor_bar_away->set_point(away_bar.first, away_bar.second);
            optimized_neighbor_bar_away->update();
        }
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

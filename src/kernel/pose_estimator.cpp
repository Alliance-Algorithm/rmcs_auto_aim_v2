#include "pose_estimator.hpp"

#include <array>
#include <numbers>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <ranges>

#include "kernel/transform_tree.hpp"
#include "module/pose_estimator/refine_strategy.hpp"
#include "utility/logging/printer.hpp"
#include "utility/math/conversion.hpp"
#include "utility/math/pose_estimator/pose_refiner.hpp"
#include "utility/math/solve_pnp/pnp_solution.hpp"
#include "utility/math/solve_pnp/reprojection.hpp"
#include "utility/math/solve_pnp/solve_pnp.hpp"
#include "utility/serializable.hpp"
#include "utility/yaml/tf.hpp"

using namespace rmcs::util;
using namespace rmcs::pose_estimator;
using namespace rmcs;

namespace rmcs::kernel {

struct PoseEstimator::Impl {

    struct Config : util::Serializable {
        std::array<float, 9> camera_matrix;
        std::array<float, 5> distort_coeff;

        constexpr static std::tuple metas {
            &Config::camera_matrix,
            "camera_matrix",
            &Config::distort_coeff,
            "distort_coeff",
        };
    };

    Config config;
    PnpSolution pnp_solution {};
    std::array<std::array<double, 3>, 3> camera_matrix_array {};
    std::array<double, 5> dist_coeff_array {};
    cv::Mat camera_matrix_cv {};
    cv::Mat dist_coeffs_cv {};

    Eigen::Vector3d odom_to_camera_translation { Eigen::Vector3d::Zero() };
    Eigen::Quaterniond odom_to_camera_orientation { Eigen::Quaterniond::Identity() };

    PoseRefiner refiner;
    Printer log { "PoseEstimator" };

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> try {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }
        {
            auto result = serialize_from<tf::AutoAim>(yaml["transforms"]);
            if (!result.has_value()
                && result.error() != SerializeTfError::UNMATCHED_LINKS_IN_TREE) {
                return std::unexpected { std::string { "Failed to parse transforms | " }
                    + util::to_string(result.error()) };
            }
        }
        {
            camera_matrix_array = reshape_array<float, 9, double, 3, 3>(config.camera_matrix);
            dist_coeff_array    = reshape_array<float, 5, double>(config.distort_coeff);
            pnp_solution.input.camera_matrix = camera_matrix_array;
            pnp_solution.input.distort_coeff = dist_coeff_array;
            camera_matrix_cv                 = cast_opencv_matrix(camera_matrix_array);
            dist_coeffs_cv                   = cast_opencv_matrix(dist_coeff_array);
        }

        return {};
    } catch (const std::exception& e) {
        return std::unexpected { e.what() };
    }

    auto solve_pnp(std::vector<Armor2D> const& armors) -> std::optional<std::vector<Armor3D>> {
        if (armors.empty()) return std::nullopt;

        auto armor_shape = [](ArmorShape shape) {
            return (shape == ArmorShape::SMALL) ? rmcs::kSmallArmorShapeOpenCV
                                                : rmcs::kLargeArmorShapeOpenCV;
        };

        auto armors_in_camera = std::vector<Armor3D> {};

        std::ranges::for_each(armors | std::views::enumerate,
            [&armors_in_camera, &armor_shape, this](auto const& item) {
                auto [i, armor] = item;

                pnp_solution.input.armor_shape = armor_shape(armor.shape);
                pnp_solution.input.genre       = armor.genre;
                pnp_solution.input.color       = armor_color2camp_color(armor.color);
                std::ranges::copy(armor.corners(), pnp_solution.input.armor_detection.begin());

                auto solved = pnp_solution.solve();
                if (!solved) {
                    log.warn("solvePnP failed for armor {} ({} {})", i, get_enum_name(armor.genre),
                        get_enum_name(armor.color));
                    return;
                }

                auto armor_3d  = Armor3D {};
                armor_3d.genre = pnp_solution.result.genre;
                armor_3d.color = camp_color2armor_color(pnp_solution.result.color);
                armor_3d.id    = i;
                pnp_solution.result.translation.copy_to(armor_3d.translation);
                pnp_solution.result.orientation.copy_to(armor_3d.orientation);

                armors_in_camera.emplace_back(armor_3d);
            });

        return armors_in_camera;
    }

    auto solve_pnp(Armor2D const& armor) -> std::optional<Armor3D> {
        auto armor_shape = [](ArmorShape shape) {
            return (shape == ArmorShape::SMALL) ? rmcs::kSmallArmorShapeOpenCV
                                                : rmcs::kLargeArmorShapeOpenCV;
        };

        pnp_solution.input.armor_shape = armor_shape(armor.shape);
        pnp_solution.input.genre       = armor.genre;
        pnp_solution.input.color       = armor_color2camp_color(armor.color);
        std::ranges::copy(armor.corners(), pnp_solution.input.armor_detection.begin());

        if (!pnp_solution.solve()) {
            log.warn("solvePnP failed for armor ({} {})", get_enum_name(armor.genre),
                get_enum_name(armor.color));
            return std::nullopt;
        }

        auto armor_3d  = Armor3D {};
        armor_3d.genre = pnp_solution.result.genre;
        armor_3d.color = camp_color2armor_color(pnp_solution.result.color);
        pnp_solution.result.translation.copy_to(armor_3d.translation);
        pnp_solution.result.orientation.copy_to(armor_3d.orientation);
        return armor_3d;
    }

    auto refine_pose(const Armor2D& armor_2d, Armor3D& armor_3d) const -> void {
        const auto strategy = RefineStrategy::choose_strategy(armor_2d.genre, armor_2d.shape);

        auto pose                 = Pose {};
        auto image_corners        = std::array<cv::Point2f, 4> {};
        auto object_points        = std::array<cv::Point3f, 4> {};
        auto const& camera_matrix = camera_matrix_cv;
        auto const& dist_coeffs   = dist_coeffs_cv;

        { // preprocess
            armor_3d.translation.copy_to(pose.translation);
            armor_3d.orientation.copy_to(pose.orientation);

            const auto local_y_in_camera = pose.orientation * Eigen::Vector3d::UnitY();
            if (local_y_in_camera.y() < 0) {
                pose.orientation = pose.orientation
                    * Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitX());
            }

            std::ranges::copy(armor_2d.corners(), image_corners.begin());

            const auto& shape_points = (armor_2d.shape == ArmorShape::SMALL)
                ? rmcs::kSmallArmorShapeOpenCV
                : rmcs::kLargeArmorShapeOpenCV;
            for (std::size_t i = 0; i < object_points.size(); ++i) {
                object_points[i] = cv::Point3f { static_cast<float>(shape_points[i].x),
                    static_cast<float>(shape_points[i].y), static_cast<float>(shape_points[i].z) };
            }
        }

        { // optimize
            auto cost_function = [&](Pose const& candidate) -> double {
                const auto rotation_mat = candidate.orientation.toRotationMatrix();
                return compute_reprojection_error(candidate.translation, rotation_mat,
                    image_corners, object_points, camera_matrix, dist_coeffs);
            };

            PoseRefiner::refine(pose, strategy.yaw_range, strategy.pitch_target,
                strategy.pitch_range, cost_function);

            armor_3d.orientation = pose.orientation;
            armor_3d.translation = pose.translation;
        }
    }

    auto odom_to_camera_pipeline(std::vector<Armor2D> const& armors) -> std::vector<Armor3D> {
        if (armors.empty()) return {};

        auto transform_camera_to_odom = [&](Armor3D armor) -> Armor3D {
            auto translation_eigen = Eigen::Vector3d {};
            armor.translation.copy_to(translation_eigen);
            armor.translation = util::opencv2ros_position(translation_eigen);

            auto orientation_eigen = Eigen::Quaterniond {};
            armor.orientation.copy_to(orientation_eigen);
            armor.orientation =
                Eigen::Quaterniond(util::opencv2ros_rotation(orientation_eigen.toRotationMatrix()));
            return armor;
        };

        auto pipeline = armors | std::views::transform([&](const Armor2D& armor) {
            return std::pair { std::cref(armor), solve_pnp(armor) };
        }) | std::views::filter([](auto const& item) { return item.second.has_value(); })
            | std::views::transform([&](auto&& item) {
                  auto armor_3d = std::move(*(item.second));
                  refine_pose(item.first.get(), armor_3d);
                  return transform_camera_to_odom(std::move(armor_3d));
              });

        return pipeline | std::ranges::to<std::vector>();
    }

    auto set_odom_to_camera_transform(Transform const& transform) -> void {
        transform.position.copy_to(odom_to_camera_translation);
        transform.orientation.copy_to(odom_to_camera_orientation);
    }

    auto transform_armor_to_odom(Armor3D& armor) -> void {
        auto position = Eigen::Vector3d {};
        armor.translation.copy_to(position);
        armor.translation = odom_to_camera_orientation * position + odom_to_camera_translation;

        auto quat = Eigen::Quaterniond {};
        armor.orientation.copy_to(quat);
        armor.orientation = odom_to_camera_orientation * quat;
    }

    auto odom_to_camera(std::span<Armor3D> armors) -> std::vector<Armor3D> {
        auto pipeline = armors | std::views::transform([&](Armor3D& armor) {
            transform_armor_to_odom(armor);
            return armor;
        });
        return pipeline | std::ranges::to<std::vector>();
    }

    auto odom_to_camera(Armor3D& armor) -> Armor3D {
        transform_armor_to_odom(armor);
        return armor;
    }
};

auto PoseEstimator::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto PoseEstimator::odom_to_camera_pipeline(std::vector<Armor2D> const& armors) const
    -> std::vector<Armor3D> {
    return pimpl->odom_to_camera_pipeline(armors);
}

auto PoseEstimator::set_odom_to_camera_transform(Transform const& transform) -> void {
    return pimpl->set_odom_to_camera_transform(transform);
}

PoseEstimator::PoseEstimator() noexcept
    : pimpl { std::make_unique<Impl>() } { }

PoseEstimator::~PoseEstimator() noexcept = default;
}

#include "pose_estimator.hpp"

#include "utility/logging/printer.hpp"
#include "utility/math/conversion.hpp"
#include "utility/math/solve_pnp/pnp_solution.hpp"
#include "utility/math/solve_pnp/solve_pnp.hpp"
#include "utility/math/solve_pnp/yaw_optimizer.hpp"
#include "utility/serializable.hpp"

using namespace rmcs::util;
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
    PnpSolution pnp_solution { };
    YawOptimizer yaw_optimizer { };

    Eigen::Vector3d odom_to_camera_translation { Eigen::Vector3d::Zero() };
    Eigen::Quaterniond odom_to_camera_orientation { Eigen::Quaterniond::Identity() };

    Printer log { "PoseEstimator" };

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> try {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        pnp_solution.input.camera.camera_matrix =
            reshape_array<float, 9, double, 3, 3>(config.camera_matrix);
        pnp_solution.input.camera.distort_coeff =
            reshape_array<float, 5, double>(config.distort_coeff);

        return { };
    } catch (const std::exception& e) {
        return std::unexpected { e.what() };
    }

    auto solve_pnp(std::vector<Armor2D> const& armors) -> std::optional<std::vector<Armor3D>> {
        if (armors.empty()) return std::nullopt;

        auto armor_shape = [](ArmorShape shape) {
            if (shape == ArmorShape::SMALL) {
                return rmcs::kSmallArmorShapeOpenCV;
            } else {
                return rmcs::kLargeArmorShapeOpenCV;
            }
        };

        auto result = std::vector<Armor3D> { };

        auto q_camera_to_odom = odom_to_camera_orientation;
        auto q_odom_to_camera = q_camera_to_odom.inverse();
        auto center_yaw       = eulers(q_camera_to_odom, 2, 1, 0)[0];

        auto& input = yaw_optimizer.input;

        input.camera                             = pnp_solution.input.camera;
        input.camera.world_to_camera_orientation = Orientation { q_odom_to_camera };
        input.camera.world_to_camera_translation = Translation { Eigen::Vector3d {
            -(q_odom_to_camera * odom_to_camera_translation).eval() } };

        for (auto&& [index, armor] : armors | std::views::enumerate) {
            pnp_solution.input.armor_shape = armor_shape(armor.shape);
            pnp_solution.input.genre       = armor.genre;
            pnp_solution.input.color       = armor_color2camp_color(armor.color);
            std::ranges::copy(armor.corners(), pnp_solution.input.armor_detection.begin());

            auto solved = pnp_solution.solve();
            if (!solved) {
                log.warn("solvePnP failed for armor {} ({} {})", index, get_enum_name(armor.genre),
                    get_enum_name(armor.color));
                continue;
            }

            auto armor_3d  = Armor3D { };
            armor_3d.genre = pnp_solution.result.genre;
            armor_3d.color = camp_color2armor_color(pnp_solution.result.color);
            armor_3d.id    = static_cast<int>(index);

            armor_3d.translation = pnp_solution.result.translation;
            armor_3d.orientation = pnp_solution.result.orientation;

            auto t_in_camera = pnp_solution.result.translation.make<Eigen::Vector3d>();
            auto t_in_world  = Eigen::Vector3d {
                (q_camera_to_odom * t_in_camera + odom_to_camera_translation).eval()
            };

            input.armor_shape  = armor_shape(armor.shape);
            input.xyz_in_world = Translation { t_in_world };
            input.center_yaw   = center_yaw;
            input.genre        = armor.genre;
            std::ranges::copy(armor.corners(), input.detected_corners.begin());

            armor_3d.orientation = yaw_optimizer.solve().orientation;

            result.emplace_back(armor_3d);
        };
        return result;
    }

    auto set_odom_to_camera_transform(Transform const& transform) -> void {
        transform.position.copy_to(odom_to_camera_translation);
        transform.orientation.copy_to(odom_to_camera_orientation);
    }

    auto odom_to_camera(Armor3D const& armor) const -> Armor3D {
        auto transformed = armor;

        auto position = Eigen::Vector3d { };
        transformed.translation.copy_to(position);
        transformed.translation =
            odom_to_camera_orientation * position + odom_to_camera_translation;

        auto quat = Eigen::Quaterniond { };
        transformed.orientation.copy_to(quat);
        transformed.orientation = odom_to_camera_orientation * quat;

        return transformed;
    }

    auto odom_to_camera(std::span<Armor3D const> armors) const -> std::vector<Armor3D> {
        auto result = std::vector<Armor3D> { };
        result.reserve(armors.size());

        for (const auto& armor : armors) {
            auto transformed = armor;

            auto position = Eigen::Vector3d { };
            transformed.translation.copy_to(position);
            transformed.translation =
                odom_to_camera_orientation * position + odom_to_camera_translation;

            auto quat = Eigen::Quaterniond { };
            transformed.orientation.copy_to(quat);
            transformed.orientation = odom_to_camera_orientation * quat;

            result.emplace_back(transformed);
        }

        return result;
    }
};

auto PoseEstimator::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto PoseEstimator::solve_pnp(std::vector<Armor2D> const& armors) const
    -> std::optional<std::vector<Armor3D>> {
    return pimpl->solve_pnp(armors);
}

auto PoseEstimator::update_camera_transform(Transform const& transform) -> void {
    return pimpl->set_odom_to_camera_transform(transform);
}

auto PoseEstimator::odom_to_camera(std::span<Armor3D const> armors) const -> std::vector<Armor3D> {
    return pimpl->odom_to_camera(armors);
}
auto PoseEstimator::odom_to_camera(Armor3D const& armor) const -> Armor3D {
    return pimpl->odom_to_camera(armor);
}

PoseEstimator::PoseEstimator() noexcept
    : pimpl { std::make_unique<Impl>() } { }

PoseEstimator::~PoseEstimator() noexcept = default;
}

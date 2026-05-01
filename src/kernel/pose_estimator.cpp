#include "pose_estimator.hpp"

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
        bool yaw_optimizer;

        constexpr static std::tuple metas { //
            &Config::camera_matrix, "camera_matrix", &Config::distort_coeff, "distort_coeff",
            &Config::yaw_optimizer, "yaw_optimizer"
        };
    };

    Config config;

    CameraFeature camera_feature;
    PnpSolution pnp_solution { };
    YawOptimizer yaw_optimizer { };

    Eigen::Vector3d camera_translation { Eigen::Vector3d::Zero() };
    Eigen::Quaterniond camera_orientation { Eigen::Quaterniond::Identity() };

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> try {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        camera_feature.camera_matrix = reshape_array<float, 9, double, 3, 3>(config.camera_matrix);
        camera_feature.distort_coeff = reshape_array<float, 5, double>(config.distort_coeff);

        pnp_solution.input.camera  = camera_feature;
        yaw_optimizer.input.camera = camera_feature;

        return { };
    } catch (const std::exception& e) {
        return std::unexpected { e.what() };
    }

    auto solve_armor(std::vector<Armor2D> const& armors) {
        constexpr auto get_shape = [](ArmorShape shape) {
            if (shape == ArmorShape::SMALL) {
                return rmcs::kSmallArmorShapeOpenCV;
            } else {
                return rmcs::kLargeArmorShapeOpenCV;
            }
        };

        const auto q_camera_to_odom = camera_orientation;
        const auto q_odom_to_camera = q_camera_to_odom.inverse();
        const auto center_yaw       = eulers(q_camera_to_odom, 2, 1, 0)[0];

        auto& input = yaw_optimizer.input;

        input.camera.camera_orientation = Orientation { q_odom_to_camera };
        input.camera.camera_translation =
            Translation { Eigen::Vector3d { -(q_odom_to_camera * camera_translation).eval() } };

        auto result = std::vector<Armor3D> { };
        for (auto&& [index, armor] : armors | std::views::enumerate) {
            auto armor_3d = Armor3D { };
            armor_3d.id   = static_cast<int>(index);

            { // pnp
                auto& input       = pnp_solution.input;
                input.armor_shape = get_shape(armor.shape);
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

                input.armor_shape  = get_shape(armor.shape);
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

    auto update_camera_transform(Transform const& transform) {
        transform.translation.copy_to(camera_translation);
        transform.orientation.copy_to(camera_orientation);
    }

    auto into_odom_link(Armor3D const& armor) const -> Armor3D {
        auto transformed = armor;

        auto position = Eigen::Vector3d { };
        transformed.translation.copy_to(position);
        transformed.translation = camera_orientation * position + camera_translation;

        auto quat = Eigen::Quaterniond { };
        transformed.orientation.copy_to(quat);
        transformed.orientation = camera_orientation * quat;

        return transformed;
    }

    auto into_odom_link(std::span<Armor3D const> armors) const {
        auto result = std::vector<Armor3D> { };
        for (const auto& armor : armors) {
            result.emplace_back(into_odom_link(armor));
        }
        return result;
    }
};

auto PoseEstimator::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto PoseEstimator::estimate_armor(std::vector<Armor2D> const& armors) const
    -> std::vector<Armor3D> {
    return pimpl->solve_armor(armors);
}

auto PoseEstimator::update_camera_transform(Transform const& transform) -> void {
    return pimpl->update_camera_transform(transform);
}

auto PoseEstimator::into_odom_link(std::span<Armor3D const> armors) const -> std::vector<Armor3D> {
    return pimpl->into_odom_link(armors);
}
auto PoseEstimator::into_odom_link(Armor3D const& armor) const -> Armor3D {
    return pimpl->into_odom_link(armor);
}

PoseEstimator::PoseEstimator() noexcept
    : pimpl { std::make_unique<Impl>() } { }

PoseEstimator::~PoseEstimator() noexcept = default;
}

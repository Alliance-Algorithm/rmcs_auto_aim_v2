#include "pose_estimator.hpp"

#include "kernel/transform_tree.hpp"
#include "utility/logging/printer.hpp"
#include "utility/math/solve_pnp/pnp_solution.hpp"
#include "utility/math/solve_pnp/solve_pnp.hpp"
#include "utility/serializable.hpp"
#include "utility/yaml/tf.hpp"

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
    PnpSolution pnp_solution {};

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
            pnp_solution.input.camera_matrix =
                reshape_array<float, 9, double, 3, 3>(config.camera_matrix);
            pnp_solution.input.distort_coeff =
                reshape_array<float, 5, double>(config.distort_coeff);
        }

        return {};
    } catch (const std::exception& e) {
        return std::unexpected { e.what() };
    }

    auto solve_pnp(std::vector<Armor2D> const& armors) noexcept
        -> std::optional<std::vector<Armor3D>> {
        if (armors.empty()) return std::nullopt;

        auto armor_shape = [](ArmorShape shape) {
            if (shape == ArmorShape::SMALL) {
                return rmcs::kSmallArmorShapeOpenCV;
            } else {
                return rmcs::kLargeArmorShapeOpenCV;
            }
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
};

auto PoseEstimator::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto PoseEstimator::solve_pnp(std::vector<Armor2D> const& armors) const noexcept
    -> std::optional<std::vector<Armor3D>> {
    return pimpl->solve_pnp(armors);
}

PoseEstimator::PoseEstimator() noexcept
    : pimpl { std::make_unique<Impl>() } { }

PoseEstimator::~PoseEstimator() noexcept = default;

}

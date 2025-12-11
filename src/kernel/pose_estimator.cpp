#include "pose_estimator.hpp"

#include "kernel/transform_tree.hpp"
#include "utility/logging/printer.hpp"
#include "utility/math/solve_pnp.hpp"
#include "utility/serializable.hpp"
#include "utility/yaml/tf.hpp"

using namespace rmcs::util;
using namespace rmcs;

namespace rmcs::kernel {

struct PoseEstimator::Impl {

    struct Config : util::Serializable {
        std::array<float, 9> camera_matrix;
        std::array<float, 5> distort_coeff;

        // clang-format off
        constexpr static std::tuple metas {
            &Config::camera_matrix, "camera_matrix",
            &Config::distort_coeff, "distort_coeff",
        };
        // clang-format on
    };
    Config config;

    std::vector<PnpSolution> pnp_solutions {};

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

        return {};
    } catch (const std::exception& e) {
        return std::unexpected { e.what() };
    }

    auto solve_pnp(std::optional<std::vector<Armor2D>> const& armors) noexcept -> void {
        if (!armors.has_value()) return;

        auto _armors = (*armors);
        pnp_solutions.reserve(_armors.size());
        pnp_solutions.resize(_armors.size());

        auto color = [](ArmorColor const& color) -> CampColor {
            if (color == ArmorColor::BLUE) return CampColor::BLUE;
            if (color == ArmorColor::RED) return CampColor::RED;
            return CampColor::UNKNOWN;
        };

        auto shape = [](ArmorShape shape) -> std::array<Point3d, 4> {
            if (shape == ArmorShape::SMALL) {
                return rmcs::kSmallArmorShapeOpenCV;
            } else {
                return rmcs::kLargeArmorShapeOpenCV;
            }
        };

        auto const _camera_matrix = reshape_array<float, 9, double, 3, 3>(config.camera_matrix);
        auto const _distort_coeff = reshape_array<float, 5, double>(config.distort_coeff);

        std::ranges::for_each(std::views::zip(_armors, pnp_solutions),
            [&color, &shape, &_camera_matrix, &_distort_coeff](auto&& pair) {
                auto const& [armor, pnp_solution] = pair;

                PnpSolution& solution = const_cast<PnpSolution&>(pnp_solution);

                solution.input.armor_shape = shape(armor.shape);
                solution.input.genre       = armor.genre;
                solution.input.color       = color(armor.color);
                std::ranges::copy(armor.corners(), solution.input.armor_detection.begin());
                solution.input.camera_matrix = _camera_matrix;
                solution.input.distort_coeff = _distort_coeff;

                solution.solve();
            });
    }

    auto visualize(RclcppNode& visual_node) -> void {
        for (auto& solution : pnp_solutions) {
            solution.visualize(visual_node);
        }
    }
};

auto PoseEstimator::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

void PoseEstimator::solve_pnp(std::optional<std::vector<Armor2D>> const& armors) const noexcept {
    return pimpl->solve_pnp(armors);
}

auto PoseEstimator::visualize(RclcppNode& visual_node) -> void {
    return pimpl->visualize(visual_node);
}

PoseEstimator::PoseEstimator() noexcept
    : pimpl { std::make_unique<Impl>() } { }

PoseEstimator::~PoseEstimator() noexcept = default;

}

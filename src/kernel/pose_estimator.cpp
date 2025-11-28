#include "pose_estimator.hpp"
#include "kernel/transform_tree.hpp"
#include "utility/logging/printer.hpp"
#include "utility/math/solve_pnp.hpp"
#include "utility/serializable.hpp"
#include "utility/yaml/eigen.hpp"

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

    Printer log { "PoseEstimator" };

    PnpSolution pnp_solution;

    auto initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> try {
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        auto read_gimbal = bool { false };
        auto read_camera = bool { false };
        for (const auto& link : yaml["transforms"]) {
            auto t = read_eigen_translation<double>(link["t"]);
            auto q = read_eigen_orientation<double>(link["q"]);

            auto iso = Eigen::Isometry3d { Eigen::Translation3d { t } * q };

            auto parent = link["parent"].as<std::string>();
            auto child  = link["child"].as<std::string>();
            /*  */ if (parent == "imu_link" && child == "gimbal_link") {
                tf::AutoAim::set_state<"gimbal_link">(iso);
                read_gimbal = true;
            } else if (parent == "gimbal_link" && child == "camera_link") {
                tf::AutoAim::set_state<"camera_link">(iso);
                read_camera = true;
            } else {
                return std::unexpected { "Unknown transform was read in the yaml" };
            }
        }
        if (!read_gimbal) {
            return std::unexpected { "Lack of tf from 'imu_link' to 'gimbal_link'" };
        }
        if (!read_camera) {
            return std::unexpected { "Lack of tf from 'gimbal_link' to 'camera_link'" };
        }
        return {};

    } catch (const std::exception& e) {
        return std::unexpected { e.what() };
    }
};

auto PoseEstimator::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

PoseEstimator::PoseEstimator() noexcept
    : pimpl { std::make_unique<Impl>() } { }

PoseEstimator::~PoseEstimator() noexcept = default;

}

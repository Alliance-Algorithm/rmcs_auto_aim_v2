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

    Printer log { "PoseEstimator" };

    PnpSolution pnp_solution;

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
};

auto PoseEstimator::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

PoseEstimator::PoseEstimator() noexcept
    : pimpl { std::make_unique<Impl>() } { }

PoseEstimator::~PoseEstimator() noexcept = default;

}

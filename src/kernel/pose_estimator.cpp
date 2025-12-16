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

    /**
     * @brief 从 YAML 节点初始化关键信息：相机内参与（可选的）变换配置。
     *
     * 从给定的 YAML 中反序列化 Impl::Config 到内部配置，并尝试反序列化 transforms 到转换树。
     * 成功时会将 config 中的 camera_matrix 和 distort_coeff 重塑并写入内部 PnP 求解器的输入。
     * 对 transforms 的解析若失败且错误为 SerializeTfError::UNMATCHED_LINKS_IN_TREE 则被容忍；其他解析错误会导致初始化失败。
     *
     * @param yaml 包含 "camera_matrix"、"distort_coeff"（在 config 内）及可选 "transforms" 的 YAML 节点。
     * @returns `void` 表示成功；否则返回包含错误描述的 `std::string`。
     *
     * 额外行为：若发生 `std::exception`，其 `what()` 文本将作为错误字符串返回。
     */
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

    /**
     * @brief 对输入的二维装甲板集合逐个执行 PnP 求解，构建相机坐标系下的三维装甲信息列表。
     *
     * 对每个装甲根据其形状、类别与颜色配置内部 PnP 输入，复制检测到的角点并调用求解器。
     * 对无法求解的装甲会被跳过（并记录警告）；成功求解的装甲会被转换为 Armor3D 并保留其索引作为 id。
     *
     * @param armors 待求解的二维装甲检测结果序列；若为空则不会执行求解。
     * @return std::optional<std::vector<Armor3D>> 包含所有成功求解得到的 Armor3D 向量；当输入为空时返回 `std::nullopt`。 */
    auto solve_pnp(std::vector<Armor2D> const& armors) -> std::optional<std::vector<Armor3D>> {
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

/**
 * @brief 使用 YAML 配置初始化 PoseEstimator 的内部实现。
 *
 * 解析并加载相机内参、畸变系数和可选的变换定义，完成内部 PnP 求解器与相关状态的配置。
 *
 * @param yaml 包含初始化参数的 YAML 节点（例如 `camera_matrix`、`distort_coeff`、`transforms` 等）。
 * @return `void` on success; `std::string` error message when initialization fails.
 */
auto PoseEstimator::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

/**
 * @brief 对一组 2D 装甲板检测结果执行 PnP 求解，获得相机坐标系下的 3D 装甲信息。
 *
 * @param armors 检测到的 Armor2D 列表；对于每个元素会尝试进行 PnP 求解以恢复其三维位姿。
 * @return std::optional<std::vector<Armor3D>> 包含成功求解出的 Armor3D 列表；当输入向量为空时返回 `std::nullopt`。
 * 
 * 每个返回的 Armor3D 包含 genre、color（已转换回 ArmorColor）、id（对应原输入的索引）、以及在相机坐标系中的平移和旋转。
 */
auto PoseEstimator::solve_pnp(std::vector<Armor2D> const& armors) const
    -> std::optional<std::vector<Armor3D>> {
    return pimpl->solve_pnp(armors);
}

/**
     * @brief 构造一个 PoseEstimator 实例并初始化其内部实现（Pimpl）。
     *
     * 初始化用于封装实现细节和运行时状态的内部 Impl 对象。
     */
    PoseEstimator::PoseEstimator() noexcept
    : pimpl { std::make_unique<Impl>() } { }

PoseEstimator::~PoseEstimator() noexcept = default;

}
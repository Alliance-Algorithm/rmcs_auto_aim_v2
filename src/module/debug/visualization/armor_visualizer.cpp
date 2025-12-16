#include "armor_visualizer.hpp"

#include "utility/rclcpp/visual/armor.hpp"
#include "utility/robot/armor.hpp"

using namespace rmcs::debug;
using VisualArmor = rmcs::util::visual::Armor;

struct ArmorShadow {
    decltype(rmcs::Armor3D::genre) genre;
    decltype(rmcs::Armor3D::color) color;
    decltype(rmcs::Armor3D::id) id;

    /**
 * @brief 比较两个 ArmorShadow 的标识字段是否相等。
 *
 * @param other 要比较的另一个 ArmorShadow 对象。
 * @return `true` 如果 `genre`、`color` 和 `id` 三个字段均相同，`false` 否则。
 */
bool operator==(ArmorShadow const& other) const = default;
    /**
 * @brief 判断两个 ArmorShadow 是否不相等。
 *
 * @param other 要比较的另一个 ArmorShadow。
 * @return `true` 如果两个 ArmorShadow 在任一可比字段（genre、color 或 id）上不同，`false` 否则。
 */
bool operator!=(ArmorShadow const& other) const { return !(*this == other); }
};

struct ArmorVisualizer::Impl final {
    /**
     * @brief 将用于可视化的 Rclcpp 节点保存为内部引用以供后续调用。
     *
     * 将传入的 `visual_node` 存为内部的可选引用，使后续的 `visualize` 调用能够访问该节点。
     *
     * @param visual_node 要用于创建和更新可视化资源的 ROS2 节点引用。
     */
    auto initialize(util::RclcppNode& visual_node) noexcept -> void {
        node = std::ref(visual_node);
    }

    /**
     * @brief 根据输入的 Armor3D 列表在已初始化的可视化节点上更新或按需重建对应的三维装甲可视化对象。
     *
     * 对于每个输入的 Armor3D，函数将检查是否需要重建该索引处的 VisualArmor（当节点未初始化、类型/颜色/ID 发生变化或尚无实例时会触发重建），
     * 并在存在实例时更新其位置和朝向。
     *
     * @param _armors 要可视化的装甲数据序列，序列中每个元素对应一个可视化实例的目标状态。
     * @return true 表示已在节点上处理并更新（或重建）了所有输入的可视化对象。
     * @return false 表示可视化节点尚未初始化，未进行任何处理。
     */
    auto visualize(std::span<Armor3D> const& _armors) -> bool {
        if (!node.has_value()) {
            return false;
        }

        auto new_size = _armors.size();
        visual_armors.reserve(new_size);
        current_armors.reserve(new_size);
        visual_armors.resize(new_size);
        current_armors.resize(new_size);

        for (size_t i = 0; i < new_size; i++) {
            auto const& input = _armors[i];
            auto& armor_ptr   = visual_armors[i];
            auto& shadow      = current_armors[i];

            bool changed = !armor_ptr || needs_rebuild(shadow, input);

            if (changed) {
                auto const config = VisualArmor::Config {
                    .rclcpp = node.value().get(),
                    .device = input.genre,
                    .camp   = armor_color2camp_color(input.color),
                    .id     = input.id,
                    .name   = "solved_pnp_armor",
                    .tf     = "camera_link",
                };

                armor_ptr = std::make_unique<VisualArmor>(config);

                shadow.genre = input.genre;
                shadow.color = input.color;
                shadow.id    = input.id;
            }

            armor_ptr->move(input.translation, input.orientation);
            armor_ptr->update();
        }

        return true;
    }

    // static auto camp(ArmorColor const& color) -> CampColor {
    //     if (color == ArmorColor::BLUE) return CampColor::BLUE;
    //     if (color == ArmorColor::RED) return CampColor::RED;
    //     return CampColor::UNKNOWN;
    /**
     * @brief 判断输入的 Armor3D 是否与当前的 ArmorShadow 在身份字段上不同，从而需要重建。
     *
     * @param shadow 当前保存的轻量化身份表示（genre、color、id）。
     * @param input 新的 Armor3D 数据用于比较身份字段。
     * @return true 如果 `genre`、`color` 或 `id` 任一字段与 shadow 不同，false 否则。
     */

    static bool needs_rebuild(ArmorShadow shadow, Armor3D const& input) {
        return shadow.genre != input.genre || shadow.color != input.color || shadow.id != input.id;
    }

    std::optional<std::reference_wrapper<util::RclcppNode>> node;
    std::vector<ArmorShadow> current_armors;
    std::vector<std::unique_ptr<VisualArmor>> visual_armors;
};

/**
 * @brief 使用给定的 ROS 节点初始化可视化器以便进行场景渲染。
 *
 * @param visual_node 用于可视化资源和发布/订阅的 rclcpp 节点引用。
 */
auto ArmorVisualizer::initialize(util::RclcppNode& visual_node) noexcept -> void {
    return pimpl->initialize(visual_node);
}

/**
 * @brief 对传入的一组三维装甲目标执行可视化更新。
 *
 * 为每个 Armor3D 条目创建或更新对应的可视化实体并应用其位姿，必要时重建可视化对象以反映身份变化。
 *
 * @param armors 待可视化的 Armor3D 对象序列（每项包含身份、颜色、位置与朝向信息）。
 * @return true 如果可视化节点已初始化且已处理所有输入装甲，false 如果可视化节点未初始化导致无法执行更新。
 */
auto ArmorVisualizer::visualize(std::span<Armor3D> const& armors) -> bool {
    return pimpl->visualize(armors);
}

/**
     * @brief 构造一个 ArmorVisualizer 实例并初始化其内部实现。
     *
     * 在构造时为内部实现分配并持有一个 Impl 的唯一所有权指针（pimpl 模式）。
     */
    ArmorVisualizer::ArmorVisualizer() noexcept
    : pimpl { std::make_unique<Impl>() } { };
/**
 * @brief 销毁 ArmorVisualizer 实例并释放其内部实现及关联资源。
 *
 * 在对象生命周期结束时清理并释放由可见化器持有的内部实现与资源。
 */
ArmorVisualizer::~ArmorVisualizer() noexcept = default;
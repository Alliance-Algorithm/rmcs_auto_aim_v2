#include "armor.hpp"
#include "utility/panic.hpp"
#include "utility/rclcpp/node.details.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

using namespace rmcs::util::visual;

using Marker      = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

struct Armor::Impl {
    static inline rclcpp::Clock rclcpp_clock { RCL_SYSTEM_TIME };

    Config config;

    Marker marker;
    Marker arrow_marker;
    std::shared_ptr<rclcpp::Publisher<MarkerArray>> rclcpp_pub;

    /**
     * @brief 使用给定的可移动配置构造 Impl 并完成内部初始化。
     *
     * 构造后会将传入的配置存入成员并执行初始化流程（包括命名校验、标记和发布器的初始设置）。
     *
     * @param config 装甲可视化的配置项（按值传入，构造时会被移动到内部存储）。
     */
    explicit Impl(Config config)
        : config(std::move(config)) {
        initialize();
    }

    /**
     * @brief 为给定配置创建并返回一个用于发布 MarkerArray 的 ROS2 发布器。
     *
     * 使用 config.rclcpp.get_pub_topic_prefix() 与 config.name 拼接生成话题名，
     * 并通过 config.rclcpp.details->make_pub 创建具有 `qos::debug` QoS 的发布器。
     *
     * @param config 包含话题前缀、名称及用于构建发布器的 rclcpp 细节的配置对象。
     * @return std::shared_ptr<rclcpp::Publisher<MarkerArray>> 指向已创建、绑定到生成话题名且使用 `qos::debug` 的 MarkerArray 发布器的 shared_ptr。
     */
    static auto create_rclcpp_publisher(Config const& config) noexcept
        -> std::shared_ptr<rclcpp::Publisher<MarkerArray>> {
        const auto topic_name { config.rclcpp.get_pub_topic_prefix() + config.name };
        return config.rclcpp.details->make_pub<MarkerArray>(topic_name, qos::debug);
    }

    /**
     * @brief 初始化并配置用于可视化的方块和箭头标记的属性。
     *
     * 对内部的 marker 与 arrow_marker 设置帧、命名空间、ID、类型、寿命、尺寸与颜色等属性，
     * 并根据配置的设备和阵营选择适当的尺寸与颜色。
     *
     * @throws std::runtime_error 若 config.name 或 config.tf 命名不符合规范时会触发 panic（终止）。
     * @throws std::runtime_error 若 config.device 不属于已知的小型或大型装甲设备集合时会触发 panic（终止）。
     */
    auto initialize() -> void {
        if (!prefix::check_naming(config.name) || !prefix::check_naming(config.tf)) {
            util::panic(std::format(
                "Not a valid naming for armor name or tf: {}", prefix::naming_standard));
        }

        marker.header.frame_id = config.tf;
        marker.ns              = config.name;
        marker.id              = config.id;
        marker.type            = Marker::CUBE;
        marker.action          = Marker::ADD;
        marker.lifetime        = rclcpp::Duration::from_seconds(0.1);

        // ref: "https://www.robomaster.com/zh-CN/products/components/detail/149"
        /*  */ if (DeviceIds::kSmallArmorDevices().contains(config.device)) {
            marker.scale.x = 0.003, marker.scale.y = 0.140, marker.scale.z = 0.125;
        } else if (DeviceIds::kLargeArmorDevices().contains(config.device)) {
            marker.scale.x = 0.003, marker.scale.y = 0.235, marker.scale.z = 0.127;
        } else {
            util::panic("Wrong device id for a visualized armor");
        };

        /*  */ if (config.camp == CampColor::RED) {
            marker.color.r = 1., marker.color.g = 0., marker.color.b = 0., marker.color.a = 1.;
        } else if (config.camp == CampColor::BLUE) {
            marker.color.r = 0., marker.color.g = 0., marker.color.b = 1., marker.color.a = 1.;
        } else {
            marker.color.r = 1., marker.color.g = 0., marker.color.b = 1., marker.color.a = 1.;
        }

        arrow_marker.header.frame_id = config.tf;
        arrow_marker.ns              = config.name + std::string("_arrow");
        arrow_marker.id              = config.id;
        arrow_marker.type            = Marker::ARROW;
        arrow_marker.action          = Marker::ADD;
        arrow_marker.lifetime        = rclcpp::Duration::from_seconds(0.1);

        arrow_marker.scale.x = 0.2;
        arrow_marker.scale.y = 0.01;
        arrow_marker.scale.z = 0.01;

        /*  */ if (config.camp == CampColor::RED) {
            arrow_marker.color.r = 1., arrow_marker.color.g = 0., arrow_marker.color.b = 0.,
            arrow_marker.color.a = 1.;
        } else if (config.camp == CampColor::BLUE) {
            arrow_marker.color.r = 0., arrow_marker.color.g = 0., arrow_marker.color.b = 1.,
            arrow_marker.color.a = 1.;
        } else {
            arrow_marker.color.r = 1., arrow_marker.color.g = 0., arrow_marker.color.b = 1.,
            arrow_marker.color.a = 1.;
        }
    }

    /**
     * @brief 将当前的标记（方块和箭头）打包为 MarkerArray 并发布到配置的 ROS 话题。
     *
     * 会在首次调用时根据配置延迟创建用于发布 MarkerArray 的发布者；在发布前为两个标记设置当前时间戳，并将箭头标记的位置对齐到方块标记。
     */
    auto update() noexcept -> void {
        if (!rclcpp_pub) {
            rclcpp_pub = create_rclcpp_publisher(config);
        }

        MarkerArray visual_marker;
        const auto current_stamp  = rclcpp_clock.now();
        marker.header.stamp       = current_stamp;
        arrow_marker.header.stamp = current_stamp;

        arrow_marker.pose = marker.pose;
        visual_marker.markers.emplace_back(marker);
        visual_marker.markers.emplace_back(arrow_marker);

        rclcpp_pub->publish(visual_marker);
    }

    /**
     * @brief 将指定的平移和朝向写入内部标记的位姿。
     *
     * 将参数 `t` 的平移分量复制到内部 `marker.pose.position`，并将参数 `q` 的旋转复制到 `marker.pose.orientation`，用于更新可视化标记的位置与朝向。
     *
     * @param t 要应用到标记位置的平移向量。
     * @param q 要应用到标记朝向的四元数/方向表示。
     */
    auto move(const Translation& t, const Orientation& q) noexcept {
        t.copy_to(marker.pose.position);
        q.copy_to(marker.pose.orientation);
    }
};

auto Armor::update() noexcept -> void { pimpl->update(); }

auto Armor::impl_move(const Translation& t, const Orientation& q) noexcept -> void {
    pimpl->move(t, q);
}

Armor::Armor(const Config& config) noexcept
    : pimpl { std::make_unique<Impl>(config) } { }

Armor::~Armor() noexcept = default;
#include "module/debug/framerate.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/shared/client.hpp"
#include "utility/shared/context.hpp"

#include <eigen3/Eigen/Geometry>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs {

using namespace rmcs::util;

class AutoAimComponent final : public rmcs_executor::Component {
public:
    /**
     * @brief 初始化 AutoAimComponent 的运行时资源并配置帧率计数器。
     *
     * 构造时创建与组件名关联的 ROS 2 节点句柄，并将内部 FramerateCounter 的计时间隔设置为 2 秒。
     */
    explicit AutoAimComponent() noexcept
        : rclcpp { get_component_name() } {

        using namespace std::chrono_literals;
        framerate.set_interval(2s);
    }

    /**
     * @brief 从 TF 更新控制状态的位姿与时间戳，并进行共享状态的接收与发送。
     *
     * 当 RMCS TF 可用时，从 CameraLink 到 OdomImu 查找变换，使用该变换填充 control_state 的
     * camera_to_odom_transform（平移与四元数方向）和 timestamp。之后执行 recv_state() 读取
     * 共享输入状态并执行 send_state() 将当前 control_state 写回共享内存。
     */
    auto update() -> void override {
        using namespace rmcs_description;
        if (rmcs_tf.ready()) [[likely]] {
            auto camera_odom =
                fast_tf::lookup_transform<rmcs_description::CameraLink, rmcs_description::OdomImu>(
                    *rmcs_tf);

            control_state.timestamp = Clock::now();

            control_state.camera_to_odom_transform.posture = camera_odom.translation();
            control_state.camera_to_odom_transform.orientation =
                Eigen::Quaterniond(camera_odom.rotation());

            //...
        }

        recv_state();
        send_state();
    }

private:
    InputInterface<rmcs_description::Tf> rmcs_tf;

    RclcppNode rclcpp;

    ControlClient::Send shm_send;
    ControlClient::Recv shm_recv;

    ControlState control_state;

    FramerateCounter framerate;

private:
    /**
     * @brief 从共享内存接收并处理来自 AutoAim 客户端的状态更新。
     *
     * 如果接收通道未打开则尝试打开并返回；若检测到更新则读取状态时间戳为本地变量。
     * 在读取后若仍报告未清除的更新标志，会记录错误并关闭 ROS 节点。
     * 当帧率计数器触发时，会记录从状态时间戳到当前时间的延迟（毫秒）及当前更新频率（Hz）。
     */
    auto recv_state() noexcept -> void {
        using Milli = std::chrono::duration<double, std::milli>;

        if (shm_recv.opened() == false) {
            shm_recv.open(util::shared_autoaim_state_name);
            return;
        }

        if (shm_recv.is_updated()) {
            auto timestamp = Stamp {};

            shm_recv.with_read([&](const auto& state) { timestamp = state.timestamp; });

            if (shm_recv.is_updated()) {
                rclcpp.error("Updated but not clear flag");
                rclcpp.shutdown();
            }

            if (framerate.tick()) {
                auto now      = Clock::now();
                auto interval = Milli { now - timestamp };
                rclcpp.info(
                    "Client recv, delay: {:.3}ms, hz: {}", interval.count(), framerate.fps());
            }
        }
    }
    /**
     * @brief 将当前 control_state 写入共享内存的控制状态通道。
     *
     * 如果写入通道尚未打开则尝试打开对应的共享内存名称并返回；若已打开则将内部的 control_state 复制到共享内存中的状态结构以提交最新控制快照。
     */
    auto send_state() noexcept -> void {
        if (shm_send.opened() == false) {
            shm_send.open(util::shared_control_state_name);
            return;
        }

        shm_send.with_write([&](ControlState& state) {
            state = control_state;

            // ...
        });
    }
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)
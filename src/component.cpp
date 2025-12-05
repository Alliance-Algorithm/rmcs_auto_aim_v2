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
    explicit AutoAimComponent() noexcept
        : rclcpp { get_component_name() } {

        using namespace std::chrono_literals;
        framerate.set_interval(2s);
    }

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

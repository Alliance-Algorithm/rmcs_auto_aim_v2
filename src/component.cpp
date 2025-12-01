#include "module/debug/framerate.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/shared/context.hpp"

#include <rmcs_executor/component.hpp>

namespace rmcs {

using namespace rmcs::util;

class AutoAimComponent final : public rmcs_executor::Component {
public:
    explicit AutoAimComponent() noexcept
        : rclcpp { get_component_name() } {

        using namespace std::chrono_literals;
        framerate.set_intetval(2s);

        if (!shm_recv.open(util::shared_memory_id)) {
            rclcpp.error("Failed to open shared memory");
        }
    }

    auto update() -> void override {
        if (shm_recv.opened() == false) {
            shm_recv.open(util::shared_memory_id);
        } else if (shm_recv.is_updated() && framerate.tick()) {

            auto context = AutoAimState {};
            shm_recv.recv(context);

            auto timestamp = context.timestamp;
            auto now       = util::Clock::now();

            using Milli   = std::chrono::duration<double, std::milli>;
            auto interval = Milli { now - timestamp };

            rclcpp.info("Client recv, delay: {:.3}, hz: {}", interval.count(), framerate.fps());
        }
    }

private:
    RclcppNode rclcpp;

    ControlClient::Send shm_send;
    ControlClient::Recv shm_recv;

    FramerateCounter framerate;
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)

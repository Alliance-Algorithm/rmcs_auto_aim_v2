#include "modules/debug/framerate.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/shared/context.hpp"
#include "utility/shared/interprocess.hpp"

#include <rmcs_executor/component.hpp>

namespace rmcs {
using Client = shm::Client<shared::Context>::Recv;

class AutoAimComponent final : public rmcs_executor::Component {
public:
    explicit AutoAimComponent() noexcept
        : rclcpp { get_component_name() } {

        using namespace std::chrono_literals;
        framerate.set_intetval(2s);

        if (!client.open(shared::id)) {
            rclcpp.error("Failed to open shared memory");
        }
    }

    auto update() -> void override {
        if (client.opened() == false) {
            client.open(shared::id);
        } else if (client.is_updated() && framerate.tick()) {

            auto context = shared::Context {};
            client.recv(context);

            auto timestamp = context.timestamp;
            auto now       = shared::Clock::now();

            using Milli   = std::chrono::duration<double, std::milli>;
            auto interval = Milli { now - timestamp };

            rclcpp.info("Client recv, delay: {:.3}, hz: {}", interval.count(), framerate.fps());
        }
    }

private:
    util::RclcppNode rclcpp;
    Client client;
    FramerateCounter framerate;
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)

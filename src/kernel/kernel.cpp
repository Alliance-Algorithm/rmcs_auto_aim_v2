#include "kernel.hpp"
#include "kernel/capturer.hpp"
#include "kernel/visualization.hpp"
#include "modules/debug/framerate.hpp"
#include "utility/coroutine/common.hpp"

using namespace rmcs::kernel;

namespace details {
using Expansion = rmcs::util::SerializableExpansion;

template <class T>
concept has_config_trait = requires { typename T::Config; };

template <class T>
concept serialable_config_trait =
    requires { requires std::derived_from<typename T::Config, Expansion>; };

template <class T>
concept can_initialize_trait = requires(T& kernel) {
    { kernel.initialize(std::declval<typename T::Config>()) };
};

template <class T>
concept has_prefix_trait = requires {
    { T::get_prefix() } -> std::convertible_to<std::string_view>;
};

}

struct AutoAim::Impl {
public:
    explicit Impl(util::Node& _node) noexcept
        : node { _node } {
        using namespace std::chrono_literals;

        node.info("AutoAim Kernel is initializing...");

        initialize_kernel(capturer);
        initialize_kernel(visualization);

        runtime_task = std::make_unique<co::task<void>>(runtime_loop());

        timer = node.create_wall_timer(1ms, [this] {
            if (!runtime_task->coroutine) {
                node.error("Runtime task is unavaliable");
            }
        });

        node.info("AutoAim Kernel is initialized");
    }

    template <class T>
    auto initialize_kernel(T& kernel) -> void {

        static_assert(details::has_config_trait<T>,
            "Type T must define a nested type 'Config' for serialize from yaml.");
        static_assert(details::serialable_config_trait<T>,
            "T::Config must derive from rmcs::util::SerializableExpansion.");
        static_assert(details::can_initialize_trait<T>,
            "Type T must have a method 'initialize(Config)' accepting its own Config type.");
        static_assert(details::has_prefix_trait<T>,
            "Type T must provide a static method 'get_prefix()' returning a string like.");

        auto config = typename T::Config {};
        auto prefix = T::get_prefix();

        if (auto ret = config.serialize(prefix, node)) {
            node.info("Serialize config of {}", prefix);
        } else {
            node.error("Failed to serialize {}", prefix);
            node.error("    - Error: {}", ret.error());
            rclcpp::shutdown();
        }
        if (auto ret = kernel.initialize(config)) {
            node.info("Initialize {}", prefix);
        } else {
            node.error("Failed to initialize {}", prefix);
            node.error("    - Error: {}", ret.error());
        }
    }

    auto runtime_loop() noexcept -> co::task<void> {
        node.info("[Main runtime loop] starts");
        for (;;) {
            if (!rclcpp::ok()) break;

            if (auto image = capturer.fetch_image()) {

                // Publish a task to infer and calculate

                // Finally transmit command
            }
            co_await std::suspend_always {};
        }
        node.info("Because the cancellation operation [runtime loop] has ended");
    }

private:
    // Context
    std::unique_ptr<co::task<void>> runtime_task;

    // ROS2
    util::Node& node;
    std::shared_ptr<rclcpp::TimerBase> timer;

    // Util
    FramerateCounter framerate {};

    // Capturer
    kernel::Capturer capturer;
    kernel::VisualizationRuntime visualization;

    // Identifier

    // Calculator
};

AutoAim::AutoAim() noexcept
    : Node { "AutoAim", util::options }
    , pimpl { std::make_unique<Impl>(*this) } { }

AutoAim::~AutoAim() noexcept = default;

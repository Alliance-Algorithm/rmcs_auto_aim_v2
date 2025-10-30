#include "kernel.hpp"
#include "kernel/capturer.hpp"
#include "modules/debug/framerate.hpp"
#include "utility/coroutine/common.hpp"

using namespace rmcs;

struct AutoAimKernel::Impl {
public:
    explicit Impl(util::Node& _node) noexcept
        : node { _node } {
        using namespace std::chrono_literals;

        _node.info("AutoAim Kernal is initializing...");

        if (auto ret = cap_runtime.initialize()) {
            node.info("Capturer was initialized.");
        } else {
            node.error("Failed to initialize capturer.");
            node.error("Error: {}.", ret.error());
            node.error("Panic now!");
            rclcpp::shutdown();
        }

        used_image_framerate.set_intetval(10s);

        runtime_task = std::make_unique<co::task<void>>(runtime_loop());

        timer = node.create_wall_timer(1ms, [this] {
            if (!runtime_task->coroutine) {
                node.error("Runtime task is unavaliable");
            }
        });
    }

    auto runtime_loop() noexcept -> co::task<void> {
        node.info("[Main runtime loop] starts");
        for (;;) {
            if (!rclcpp::ok()) break;

            if (auto image = cap_runtime.fetch_image()) {

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
    FramerateCounter used_image_framerate {};

    // Capturer
    kernel::CapRuntime cap_runtime;

    // Identifier

    // Calculator
};

AutoAimKernel::AutoAimKernel() noexcept
    : Node { "AutoAim", util::options }
    , pimpl { std::make_unique<Impl>(*this) } { }

AutoAimKernel::~AutoAimKernel() noexcept = default;

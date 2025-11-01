#include "kernel/kernel.hpp"
#include "kernel/kernel.config.hpp"
#include "kernel/kernel.util.hpp"
#include "utility/thread/spsc_queue.hpp"

#include "kernel/capturer.hpp"
#include "kernel/visualization.hpp"

using namespace rmcs::kernel;

struct AutoAim::Impl {
public:
    explicit Impl(util::Node& _node) noexcept
        : node { _node } {

        node.info("AutoAim Kernel is initializing...");

        if (auto ret = config.serialize("", node); !ret) {
            node.error("Failed to read kernel config.");
            node.error("  e: {}", ret.error());
            rclcpp::shutdown();
        }

        initialize_kernel(capturer, node);

        // initialize_kernel(kernel1, node);
        // initialize_kernel(kernel2, node);

        if (config.use_visualization) {
            initialize_kernel(visualization, node);
        }

        using namespace std::chrono_literals;
        timer = node.create_wall_timer(1ms, [this] { kernel_event_loop(); });

        node.info("AutoAim Kernel is initialized");
    }

    auto switch_to_kernel() noexcept {
        struct awaitable {
            decltype(coroutines)& coroutines_ref;
            bool pushed = false;

            auto await_suspend(handle_type co) noexcept {
                // Resume immediately if push failed
                return pushed = coroutines_ref.push(co);
            }
            auto await_resume() const noexcept { return pushed; }

            static constexpr auto await_ready() noexcept { return false; }
        };
        return awaitable { coroutines };
    }

    /// @NOTE: The main loop for auto aiming
    ///  - event 1: query image from capturer and publish task
    ///  - event 2: resume avaliable task and clear them
    auto kernel_event_loop() noexcept -> void {
        // Publish task here
        if (auto image = capturer.fetch_image()) {
            auto task = make_consumption(std::move(image));
            tasks.emplace_back(task);
        }
        // Resume avaliable coroutine task
        std::coroutine_handle<> to_resume;
        while (coroutines.pop(to_resume)) {
            to_resume.resume();
        }
        // Check and remove finished task
        using task_type  = co::task<result_type>;
        const auto check = [this](task_type& task) {
            if (!task.done()) return false;
            if (auto& ret = task.result(); !ret) {
                node.error("Failed to exec task");
                node.error("    e: {}", ret.error());
            }
            task.destroy();
            return true;
        };
        std::erase_if(tasks, check);
    }

    /// @NOTE:
    ///  - Use "switch_to_kernel" to come back from other thread
    ///  - No blocking function on kernel context after switching
    auto make_consumption(std::unique_ptr<Image> image) noexcept //
        -> co::task<result_type> {

        // ...

        if (config.use_visualization) {
            if (!visualization.send_image(*image)) {
                // ...
            }
        }

        co_return {};
    }

private:
    // ROS2
    util::Node& node;
    std::shared_ptr<rclcpp::TimerBase> timer;

    // Context
    AutoAimConfig config;

    util::spsc_queue<handle_type, 20> coroutines;
    std::vector<co::task<result_type>> tasks;

    // Capturer
    kernel::Capturer capturer;

    // Identifier

    // Calculator

    // Debug
    kernel::VisualizationRuntime visualization;
};

AutoAim::AutoAim() noexcept
    : Node { "AutoAim", util::options }
    , pimpl { std::make_unique<Impl>(*this) } { }

AutoAim::~AutoAim() noexcept = default;

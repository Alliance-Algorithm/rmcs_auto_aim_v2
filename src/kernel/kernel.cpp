#include "kernel.impl.hpp"
#include "modules/capturer/hikcamera.hpp"
#include "modules/capturer/video.hpp"

using namespace rmcs;

struct runtime {
    using capture_t = capturer::CameraCapturer;
    using identifier_t = void;
    using tracker_t = void;
    using kernel_t = Kernel<capture_t, identifier_t, tracker_t>;

    static auto setup() -> std::unique_ptr<kernel_t> {
        auto capturer = std::make_unique<capture_t>();

        return std::make_unique<kernel_t>(std::move(capturer));
    }
};
struct develop {
    using capture_t = capturer::VideoCapturer;
    using identifier_t = void;
    using tracker_t = void;
    using kernel_t = Kernel<capture_t, identifier_t, tracker_t>;

    static auto setup() -> std::unique_ptr<kernel_t> {
        auto capturer = std::make_unique<capture_t>();

        return std::make_unique<kernel_t>(std::move(capturer));
    }
};

struct AutoAimKernel::Impl {
public:
    using mode = runtime;

    explicit Impl() noexcept
        : kernel_{mode::setup()} {}

    auto run() -> void {}

private:
    std::unique_ptr<mode::kernel_t> kernel_;
};

AutoAimKernel::AutoAimKernel() noexcept
    : pimpl{std::make_unique<Impl>()} {}

AutoAimKernel::~AutoAimKernel() noexcept = default;

auto AutoAimKernel::run() -> void { pimpl->run(); }

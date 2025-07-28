#include <rclcpp/node.hpp>

namespace rmcs {

class AutoAimKernel final {
public:
    explicit AutoAimKernel(rclcpp::Node&) noexcept;
    ~AutoAimKernel() noexcept;

    AutoAimKernel(const AutoAimKernel&) = delete;
    AutoAimKernel& operator=(const AutoAimKernel&) = delete;

    auto run() -> void;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rmcs
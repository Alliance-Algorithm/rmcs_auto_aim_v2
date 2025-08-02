#include <memory>

namespace rmcs {

class AutoAimKernel final {
public:
    explicit AutoAimKernel() noexcept;
    ~AutoAimKernel() noexcept;

    AutoAimKernel(const AutoAimKernel&) = delete;
    AutoAimKernel& operator=(const AutoAimKernel&) = delete;

    auto run() -> void;
    auto set_config(const std::string& path) noexcept -> void;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rmcs

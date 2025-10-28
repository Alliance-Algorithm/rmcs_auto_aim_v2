#include "utility/node.hpp"
#include <memory>

namespace rmcs {

class AutoAimKernel final : public util::Node {
public:
    explicit AutoAimKernel() noexcept;
    ~AutoAimKernel() noexcept override;

    AutoAimKernel(const AutoAimKernel&)            = delete;
    AutoAimKernel& operator=(const AutoAimKernel&) = delete;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rmcs

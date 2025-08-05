#include "kernel.hpp"

using namespace rmcs;

struct AutoAimKernel::Impl {
public:
    explicit Impl() noexcept { (void)this; }

    auto run(this auto&& self) -> void { (void)self; }

private:
};

AutoAimKernel::AutoAimKernel() noexcept
    : pimpl{std::make_unique<Impl>()} {}

AutoAimKernel::~AutoAimKernel() noexcept = default;

auto AutoAimKernel::run() -> void { pimpl->run(); }

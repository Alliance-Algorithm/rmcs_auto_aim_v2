#include "visualization.hpp"
#include "modules/debug/visualization/stream_session.hpp"

using namespace rmcs::kernel;

struct VisualizationRuntime::Impl {
    std::unique_ptr<debug::StreamSession> session;

    auto initialize(const Config&) noexcept -> std::expected<void, std::string> { }
};

auto VisualizationRuntime::initialize(const Config& config) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(config);
}

VisualizationRuntime::VisualizationRuntime() noexcept
    : pimpl { std::make_unique<Impl>() } { }

VisualizationRuntime::~VisualizationRuntime() noexcept = default;

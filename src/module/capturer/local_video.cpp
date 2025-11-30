#include "local_video.hpp"
using namespace rmcs::cap;

struct LocalVideo::Impl { };

auto LocalVideo::configure(const ConfigDetail& config) noexcept
    -> std::expected<void, std::string> { }

auto LocalVideo::connect() noexcept -> std::expected<void, std::string> { }

auto LocalVideo::connected() const noexcept -> bool { }

auto LocalVideo::wait_image() -> std::expected<std::unique_ptr<Image>, std::string> { }

LocalVideo::LocalVideo() noexcept
    : pimpl { std::make_unique<Impl>() } { }

LocalVideo::~LocalVideo() noexcept = default;

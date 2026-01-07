#include "image.details.hpp"

using namespace rmcs;

struct Image::Impl {
    Clock::time_point timestamp;
    Details details;
};

auto Image::details() noexcept -> Details& { return pimpl->details; }
auto Image::details() const noexcept -> const Details& { return pimpl->details; }

auto Image::get_timestamp() const noexcept -> Clock::time_point //
{
    return pimpl->timestamp;
}
auto Image::set_timestamp(Clock::time_point timestamp) noexcept -> void //
{
    pimpl->timestamp = timestamp;
}

Image::Image() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Image::~Image() noexcept = default;

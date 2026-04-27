#include "image.details.hpp"

using namespace rmcs;

struct Image::Impl {
    TimePoint timestamp;
    Details details;
};

auto Image::details() noexcept -> Details& { return pimpl->details; }
auto Image::details() const noexcept -> const Details& { return pimpl->details; }

auto Image::get_timestamp() const noexcept -> TimePoint //
{
    return pimpl->timestamp;
}
auto Image::set_timestamp(TimePoint timestamp) noexcept -> void //
{
    pimpl->timestamp = timestamp;
}

auto Image::clone() const noexcept -> std::unique_ptr<Image> {
    auto result = std::make_unique<Image>();

    result->details().mat = pimpl->details.mat.clone();
    result->set_timestamp(pimpl->timestamp);

    return result;
}

Image::Image() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Image::~Image() noexcept = default;

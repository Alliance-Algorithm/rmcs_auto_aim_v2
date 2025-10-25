#include "image.impl.hpp"

using namespace rmcs;

struct Image::Impl {
    Details details;
};

auto Image::details() noexcept -> Details& { return pimpl->details; }

auto Image::timestamp() const noexcept -> TimePoint { }

Image::Image() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Image::~Image() noexcept = default;

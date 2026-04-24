#pragma once
#include "utility/clock.hpp"
#include "utility/pimpl.hpp"

namespace rmcs {

class Image {
    RMCS_PIMPL_DEFINITION(Image)

public:
    struct Details;
    auto details() noexcept -> Details&;
    auto details() const noexcept -> Details const&;

    auto get_timestamp() const noexcept -> TimePoint;
    auto set_timestamp(TimePoint) noexcept -> void;
};

}

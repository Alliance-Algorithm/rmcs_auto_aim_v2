#pragma once
#include "utility/clock.hpp"
#include "utility/pimpl.hpp"

namespace rmcs {

class Image {
    RMCS_PIMPL_DEFINITION(Image)

public:
    using Clock = util::Clock;

    struct Details;
    auto details() noexcept -> Details&;
    auto details() const noexcept -> Details const&;

    auto get_timestamp() const noexcept -> Clock::time_point;
    auto set_timestamp(Clock::time_point) noexcept -> void;
};

}

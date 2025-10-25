#pragma once
#include "utility/pimpl.hpp"
#include <chrono>

namespace rmcs {

class Image {
    RMCS_PIMPL_DEFINITION(Image)

public:
    struct Details;
    auto details() noexcept -> Details&;

    using TimePoint = std::chrono::steady_clock::time_point;
    auto timestamp() const noexcept -> TimePoint;
};

}

#pragma once
#include "utility/pimpl.hpp"

namespace rmcs {

class Image {
    RMCS_PIMPL_DEFINITION(Image)

public:
    struct Details;
    auto details() noexcept -> Details&;
};

}

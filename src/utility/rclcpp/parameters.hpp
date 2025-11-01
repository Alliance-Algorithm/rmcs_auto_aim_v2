#pragma once
#include "utility/pimpl.hpp"

namespace rmcs::util {

struct Parameters {
    RMCS_PIMPL_DEFINITION(Parameters)

public:
    static auto share_location() noexcept -> std::string;
};

}

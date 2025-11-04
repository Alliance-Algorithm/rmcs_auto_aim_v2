#pragma once
#include "utility/serializable.hpp"

namespace rmcs::runtime {

struct AutoAimConfig : util::Serializable {

    util::flag_t use_visualization = false;

    static constexpr std::tuple metas {
        &AutoAimConfig::use_visualization,
        "use_visualization",
    };
};

}

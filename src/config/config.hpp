#pragma once
#include "utility/serializable.hpp"
#include <string>

namespace rmcs {

struct Config {

    util::string_t image_source = "camera";
    util::flag_t record_image   = false;
    util::integer_t exposure_ms = 3;

    auto serialize(std::string const& prefix, auto& node) noexcept {
        using namespace rmcs::util;
        constexpr auto s = Serializable {
            MemberMeta { &Config::image_source, "image_source" },
            MemberMeta { &Config::record_image, "record_image" },
            MemberMeta { &Config::exposure_ms, "exposure_ms" },
        };
        return s.serialize(prefix, node, *this);
    }
};

}

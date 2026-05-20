#pragma once

#include "utility/image/image.hpp"

#include <string>

namespace rmcs::util {

auto draw_text(Image&, const std::string& text) noexcept -> void;

}

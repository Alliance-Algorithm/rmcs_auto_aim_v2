#pragma once
#include "utility/image/image.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::util {

auto draw(Image&, const Armor2D&) noexcept -> void;

}

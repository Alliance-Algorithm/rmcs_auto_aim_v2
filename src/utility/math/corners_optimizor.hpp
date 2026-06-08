#pragma once
#include "utility/image/image.hpp"
#include "utility/robot/armor.hpp"

namespace rmcs::util {

auto optimize_corners(const Image& image, Armor2ds& armors) -> void;

auto optimize_corners(const Image& image, Lightbar2d& lightbar) -> void;

}

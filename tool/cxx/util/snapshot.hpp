#pragma once

#include <string>
#include <string_view>

namespace rmcs::tool::util {

auto build_snapshot_path(std::string_view prefix = "hikcamera") -> std::string;

} // namespace rmcs::tool::util

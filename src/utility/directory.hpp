#pragma once
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace rmcs {

auto get_directory() -> std::string {
    return ament_index_cpp::get_package_share_directory(PACKAGE_NAME);
}

} // namespace rmcs

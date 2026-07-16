#include "parameters.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace rmcs::util {

auto Parameters::share_location() noexcept -> std::string {
    return ament_index_cpp::get_package_share_directory("rmcs_auto_aim_v2");
}
}

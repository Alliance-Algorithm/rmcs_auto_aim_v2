#include "snapshot.hpp"

#include <ctime>

namespace rmcs::tool::util {

auto build_snapshot_path(std::string_view prefix) -> std::string {
    const auto now = std::time(nullptr);
    auto tm        = std::tm { };
    (void)localtime_r(&now, &tm);

    char timestamp[32] = { };
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d-%H%M%S", &tm);

    return std::string("/tmp/") + std::string(prefix) + "-" + timestamp + ".png";
}

} // namespace rmcs::tool::util

#include <chrono>
#include <ctime>

namespace rmcs {

auto function() -> void {
    static_assert(
        sizeof(std::time_t) == sizeof(std::chrono::high_resolution_clock::time_point), "");
}

} // namespace rmcs
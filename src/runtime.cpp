#include "kernel/kernel.hpp"
#include "utility/directory.hpp"
#include <print>

using namespace rmcs;

auto main() -> int {
    std::println("Hello World!!");
    std::println("Path: {}", get_directory());
}

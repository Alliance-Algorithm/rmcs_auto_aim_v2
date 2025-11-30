#include "panic.hpp"
#include <chrono>
#include <cstdlib>
#include <execinfo.h>
#include <iostream>
#include <source_location>
#include <thread>
#include <unistd.h>

namespace rmcs::util {

auto panic(const std::string& message, const std::source_location& loc) -> void {

    constexpr auto panic_head = "\033[31m========== PANIC HEAD ==========\033[0m";
    constexpr auto panic_tail = "\033[31m========== PANIC TAIL ==========\033[0m";

    using namespace std::chrono;

    auto now   = system_clock::now();
    auto now_c = system_clock::to_time_t(now);

    std::cerr << '\n' << panic_head << '\n';

    std::cerr << "  Message: \033[93m" << message << "\033[0m\n";
    std::cerr << "     File: " << loc.file_name() << '\n';
    std::cerr << " Function: " << loc.function_name() << '\n';
    std::cerr << "     Line: " << loc.line() << '\n';
    std::cerr << "   Column: " << loc.column() << '\n';
    std::cerr << "Thread ID: " << std::this_thread::get_id() << '\n';
    std::cerr << "Timestamp: " << std::ctime(&now_c);

    void* callstack[64];
    auto frames  = ::backtrace(callstack, 64);
    auto symbols = ::backtrace_symbols(callstack, frames);

    std::cerr << "\nStack trace (" << frames << " frames):\n";
    for (int i = 0; i < frames; ++i) {
        std::cerr << "  [" << i << "] " << symbols[i] << '\n';
    }
    std::free(symbols);

    std::cerr << panic_tail << std::endl;

    std::abort();
}

} // namespace rmcs::util

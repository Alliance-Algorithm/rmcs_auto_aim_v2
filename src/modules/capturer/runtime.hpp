#pragma once
#include "modules/capturer/common.hpp"

#include <thread>

namespace rmcs::cap {

template <class Impl>
class Runtime final : Adapter<Impl> {

    std::jthread runtime_worker;

    explicit Runtime() noexcept
        : Adapter<Impl> {} {
        runtime_worker = std::jthread {
            [this](const std::stop_token& token) { },
        };
    }
    ~Runtime() noexcept {
        runtime_worker.request_stop();
        if (runtime_worker.joinable()) {
            runtime_worker.join();
        }
    }
    auto await_image() noexcept -> void;

    // TODO: 手动注册挂起点到 time callback？
    auto notify() noexcept -> void { }
};

}

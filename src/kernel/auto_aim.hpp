#pragma once

#include "utility/pimpl.hpp"
#include "utility/shared/context.hpp"

#include <atomic>
#include <mutex>

namespace rmcs {

class AutoAim {
    RMCS_PIMPL_DEFINITION(AutoAim)

public:
    template <typename WithFunc>
        requires std::invocable<WithFunc, SystemContext&>
    auto with_context(WithFunc&& func) {
        std::lock_guard lock(context_mutex);
        func(current_context);
    }

    template <typename WithFunc>
        requires std::invocable<WithFunc, const AutoAimState&>
    auto with_command(WithFunc&& func) {
        std::lock_guard lock(command_mutex);
        func(current_command);
    }

    auto command_updated() -> bool {
        return unread_command.exchange(false, std::memory_order::acquire);
    }

private:
    std::mutex context_mutex;
    SystemContext current_context { SystemContext::kIdentity() };

    std::mutex command_mutex;
    std::atomic<bool> unread_command = false;
    AutoAimState current_command { AutoAimState::kInvalid() };
};

}

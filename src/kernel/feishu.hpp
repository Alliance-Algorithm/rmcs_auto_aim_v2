#pragma once
#include "utility/pimpl.hpp"
#include "utility/shared/context.hpp"

namespace rmcs::kernel {

class Feishu {
    RMCS_PIMPL_DEFINITION(Feishu)

public:
    using AutoAimState = util::AutoAimState;
    using ControlState = util::ControlState;

    auto update_state(const AutoAimState&) noexcept -> void;

    auto updated() const noexcept -> bool;

    auto system_state() const noexcept -> const ControlState&;
};

}

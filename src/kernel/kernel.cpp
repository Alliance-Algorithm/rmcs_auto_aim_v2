#include "kernel/utility/pimpl.hpp"

namespace rmcs {

class AutoAimKernel final {
    RMCS_PIMPL_DEFINITION(AutoAimKernel)

public:
    auto run() -> void;
};

} // namespace rmcs
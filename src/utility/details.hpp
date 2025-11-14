#pragma once
#include <memory>

namespace details::internal {
inline auto use_memory_header() {
    // Remove warning from unused include
    std::ignore = std::unique_ptr<int>();
}
} // namespace internal

#define RMCS_DETAILS_DEFINITION(CLASS)                                                             \
public:                                                                                            \
    explicit CLASS() noexcept;                                                                     \
    ~CLASS() noexcept;                                                                             \
    CLASS(const CLASS&)            = delete;                                                       \
    CLASS& operator=(const CLASS&) = delete;                                                       \
                                                                                                   \
    struct Details;                                                                                \
    std::unique_ptr<Details> details;

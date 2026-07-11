#pragma once

#include <memory> // IWYU pragma: keep

// NOLINTBEGIN(bugprone-macro-parentheses)
#define RMCS_PIMPL_DEFINITION(CLASS)                                                               \
public:                                                                                            \
    explicit CLASS() noexcept;                                                                     \
    ~CLASS() noexcept;                                                                             \
    CLASS(const CLASS&)            = delete;                                                       \
    CLASS& operator=(const CLASS&) = delete;                                                       \
                                                                                                   \
private:                                                                                           \
    struct Impl;                                                                                   \
    std::unique_ptr<Impl> pimpl;
// NOLINTEND(bugprone-macro-parentheses)

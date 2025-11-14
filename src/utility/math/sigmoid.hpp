#pragma once
#include <cmath>

namespace rmcs::util {

template <typename T>
inline T sigmoid(T x) noexcept {
    if (x >= T { 0 }) {
        const auto z = std::exp(-x);
        return T { 1 } / (T { 1 } + z);
    }

    const auto z = std::exp(x);
    return z / (T { 1 } + z);
}

}

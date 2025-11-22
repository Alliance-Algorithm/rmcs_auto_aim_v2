#pragma once
#include "utility/math/linear.hpp"
#include <tuple>

namespace rmcs::util::visual {

struct Movable {
    auto move(this auto& self, const Translation& t, const Orientation& q) noexcept {
        self.impl_move(t, q);
    }
    auto move(this auto& self, const std::tuple<Translation, Orientation>& tuple) noexcept {
        self.impl_move(std::get<0>(tuple), std::get<1>(tuple));
    }
    auto move(this auto& self, const translation_trait auto& t,
        const orientation_trait auto& q) noexcept {
        self.impl_move(Translation { t }, Orientation { q });
    }
};

}

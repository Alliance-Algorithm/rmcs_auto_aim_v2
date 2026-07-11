#pragma once
#include "utility/math/linear.hpp"

#include <algorithm>
#include <ranges>
#include <vector>

namespace rmcs {

struct AimPoint : Point3d {
    bool valid = true;

    constexpr AimPoint() noexcept = default;

    constexpr AimPoint(double x, double y, double z, bool valid = true) noexcept
        : Point3d { x, y, z }
        , valid { valid } { }

    constexpr explicit AimPoint(const scalar3d_trait auto& point, bool valid = true) noexcept
        : Point3d { point }
        , valid { valid } { }
};

struct AimPoints : std::vector<AimPoint> {
    using vector::vector;

    // 目前只有 Rune 需要有效性判断，为简化 Robot 和 Outpost 的实现逻辑，
    // 保留此显式转换以兼容接口
    explicit AimPoints(const std::vector<Point3d>& points) {
        vector::reserve(points.size());
        for (const auto point : points) {
            vector::emplace_back(point);
        }
    }

    auto same_valid(const AimPoints& o) const -> bool {
        if (vector::size() != o.size()) {
            return false;
        }
        return std::ranges::all_of(std::views::zip(*this, o), [](const auto& pair) {
            const auto& [p1, p2] = pair;
            return p1.valid == p2.valid;
        });
    }

    auto valid_indices() const {
        constexpr auto pred = [](const auto& pair) { return std::get<1>(pair).valid; };
        return std::ranges::to<std::vector<std::size_t>>(
            *this | std::views::enumerate | std::views::filter(pred) | std::views::keys);
    }
};

}

#pragma once

#include <eigen3/Eigen/Core>

#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace rmcs::util {

// Solves a square minimum-cost assignment matrix.
template <typename Scalar>
auto hungarian(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& cost)
    -> std::vector<int> {
    if (cost.rows() != cost.cols()) {
        throw std::invalid_argument { "hungarian() requires a square cost matrix" };
    }

    const auto n = static_cast<int>(cost.rows());
    if (n == 0) return {};

    auto u = std::vector<Scalar>(n + 1, 0);
    auto v = std::vector<Scalar>(n + 1, 0);
    auto p = std::vector<int>(n + 1, 0);
    auto w = std::vector<int>(n + 1, 0);

    for (int i = 1; i <= n; ++i) {
        p[0]     = i;
        auto j0  = 0;
        auto min = std::vector<Scalar>(n + 1, std::numeric_limits<Scalar>::max());
        auto used = std::vector<char>(n + 1, false);

        do {
            used[j0]          = true;
            auto i0           = p[j0];
            Scalar delta      = std::numeric_limits<Scalar>::max();
            auto j1           = 0;

            for (int j = 1; j <= n; ++j) {
                if (!used[j]) {
                    auto cur = static_cast<Scalar>(cost(i0 - 1, j - 1)) - u[i0] - v[j];
                    if (cur < min[j]) {
                        min[j] = cur;
                        w[j]   = j0;
                    }
                    if (min[j] < delta) {
                        delta = min[j];
                        j1    = j;
                    }
                }
            }

            for (int j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    min[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            auto j1 = w[j0];
            p[j0]   = p[j1];
            j0      = j1;
        } while (j0 != 0);
    }

    auto result = std::vector<int>(n, -1);
    for (int j = 1; j <= n; ++j)
        if (p[j] != 0) result[p[j] - 1] = j - 1;
    return result;
}

template <typename Scalar>
auto hungarian_assign(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& cost,
    Scalar gate_threshold) -> std::vector<std::optional<int>> {

    const auto N = static_cast<int>(cost.rows());
    const auto M = static_cast<int>(cost.cols());

    if (N == 0) return {};
    if (M == 0) return std::vector<std::optional<int>>(N, std::nullopt);

    const auto K      = N + M;
    const auto kHuge  = std::numeric_limits<Scalar>::max() / 4;

    auto aug = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>(K, K);
    aug.setConstant(kHuge);

    aug.topLeftCorner(N, M) = cost;

    for (int i = 0; i < N; ++i)
        for (int j = M; j < K; ++j)
            aug(i, j) = (j - M == i) ? gate_threshold : kHuge;

    for (int i = N; i < K; ++i)
        for (int j = 0; j < M; ++j)
            aug(i, j) = (i - N == j) ? Scalar { 0 } : kHuge;

    for (int i = N; i < K; ++i)
        for (int j = M; j < K; ++j)
            aug(i, j) = 0;

    auto assignment = hungarian(aug);

    auto result = std::vector<std::optional<int>>(N, std::nullopt);
    for (int i = 0; i < N; ++i) {
        auto j = assignment[i];
        if (j >= 0 && j < M) result[i] = j;
    }
    return result;
}

}

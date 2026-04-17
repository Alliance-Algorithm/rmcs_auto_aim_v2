#pragma once

#include <cmath>
#include <optional>

#include <eigen3/Eigen/Cholesky>

namespace rmcs::util {

template <class TVec, class TMat>
inline auto mahalanobis_distance(TVec const& innovation, TMat const& covariance)
    -> std::optional<double> {
    auto solved   = covariance.ldlt().solve(innovation);
    auto distance = innovation.dot(solved);
    if (!std::isfinite(distance)) return std::nullopt;
    return distance;
}

}

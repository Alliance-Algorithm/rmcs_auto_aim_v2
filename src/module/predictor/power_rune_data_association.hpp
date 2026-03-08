#pragma once

#include <cmath>
#include <limits>
#include <numbers>
#include <optional>

#include <eigen3/Eigen/Cholesky>

#include "utility/math/angle.hpp"

namespace rmcs::predictor {

template <typename TEKF>
struct PowerRuneAssociationResult {
    typename TEKF::ZVec best_z;
    double innovation_d2;
};

struct PowerRuneDataAssociator {
    static constexpr auto kBladeSymmetry  = 5;
    static constexpr auto kBladePhaseStep = 2.0 * std::numbers::pi / 5.0;
    // Chi-square gate for 7D innovation at ~99% confidence.
    static constexpr auto kInnovationGateThreshold = 18.48;

    template <typename TEKF, typename TZSubtract>
    static auto associate(typename TEKF::ZVec const& z_raw, double blade_angle_obs,
        typename TEKF::ZVec const& h_x, typename TEKF::HMat const& H,
        typename TEKF::PMat const& P, typename TEKF::RMat const& R, TZSubtract&& z_subtract)
        -> std::optional<PowerRuneAssociationResult<TEKF>> {
        const auto S           = H * P * H.transpose() + R;
        const auto Sinv_solver = S.ldlt();
        if (Sinv_solver.info() != Eigen::Success) {
            return std::nullopt;
        }

        auto best_z  = z_raw;
        auto best_d2 = std::numeric_limits<double>::infinity();
        for (int sector = -(kBladeSymmetry / 2); sector <= (kBladeSymmetry / 2); ++sector) {
            auto z_candidate = z_raw;
            z_candidate(3)   = util::normalize_angle(
                blade_angle_obs + static_cast<double>(sector) * kBladePhaseStep);
            const auto innovation = z_subtract(z_candidate, h_x);
            const auto d2         = innovation.dot(Sinv_solver.solve(innovation));
            if (std::isfinite(d2) && d2 < best_d2) {
                best_d2 = d2;
                best_z  = z_candidate;
            }
        }

        if (!std::isfinite(best_d2) || best_d2 > kInnovationGateThreshold) {
            return std::nullopt;
        }

        return PowerRuneAssociationResult<TEKF> { best_z, best_d2 };
    }
};

} // namespace rmcs::predictor

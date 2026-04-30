#include "module/fire_control/planner/reference_trajectory_builder.hpp"

#include <array>
#include <chrono>
#include <format>

#include "utility/math/angle.hpp"

namespace rmcs::fire_control {

auto ReferenceTrajectoryBuilder::build(
    TimePoint center_time, AimAttitudeSampler const& sample_attitude) const
    -> std::expected<ReferenceTrajectory, std::string> {
    constexpr int kCenterSampleIndex = (kMpcAxisHorizon / 2) + 1;

    auto samples = std::array<AimAttitude, kMpcAxisHorizon + 2> {};
    for (int i = 0; i < static_cast<int>(samples.size()); ++i) {
        auto const offset = (i - kCenterSampleIndex) * kMpcAxisDt;
        auto const t      = center_time
            + std::chrono::duration_cast<rmcs::Duration>(std::chrono::duration<double> { offset });
        auto sample = sample_attitude(t);
        if (!sample.has_value()) {
            return std::unexpected { std::format(
                "reference sample failed at index {}: {}", i, sample.error()) };
        }
        samples[i] = *sample;
    }

    auto trajectory = ReferenceTrajectory {};
    for (int i = 0; i < kMpcAxisHorizon; ++i) {
        auto const& previous = samples[i];
        auto const& current  = samples[i + 1];
        auto const& next     = samples[i + 2];

        auto const yaw_velocity =
            rmcs::util::normalize_angle(next.yaw - previous.yaw) / (2.0 * kMpcAxisDt);
        auto const pitch_velocity = (next.pitch - previous.pitch) / (2.0 * kMpcAxisDt);

        trajectory.col(i) << current.yaw, yaw_velocity, current.pitch, pitch_velocity;
    }

    return trajectory;
}

} // namespace rmcs::fire_control

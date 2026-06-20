#include "module/fire_control/planner/reference_builder.hpp"

#include <array>
#include <chrono>
#include <format>

#include "utility/math/angle.hpp"

using namespace rmcs::fire_control;

struct ReferenceBuilder::Impl {
    TargetSolver sampler {};

    auto build(predictor::Snapshot const& snapshot, TargetSolution const& raw_solution,
        TimePoint command_time, double bullet_speed, double shoot_delay) const
        -> std::expected<ReferenceTrajectory, std::string> {

        auto samples = std::array<AimAttitude, kMpcHorizon + 2> {};
        for (int i = 0; i < static_cast<int>(samples.size()); ++i) {
            auto const offset = i * kMpcDt;
            auto const t      = command_time
                + std::chrono::duration_cast<rmcs::Duration>(
                    std::chrono::duration<double> { offset });

            if (i == 0) {
                samples[i] = raw_solution.attitude;
                continue;
            }

            auto solution = sampler.solve(
                snapshot, raw_solution.candidate.armor.id, t, bullet_speed, shoot_delay);
            if (!solution.has_value()) {
                return std::unexpected { std::format(
                    "reference sample failed at index {}: {}", i, solution.error()) };
            }
            samples[i] = solution->attitude;
        }

        auto trajectory = ReferenceTrajectory {};
        for (int i = 0; i < kMpcHorizon; ++i) {
            auto const& previous = samples[i];
            auto const& current  = samples[i + 1];
            auto const& next     = samples[i + 2];

            auto const yaw_velocity =
                util::normalize_angle(next.yaw - previous.yaw) / (2.0 * kMpcDt);
            auto const pitch_velocity = (next.pitch - previous.pitch) / (2.0 * kMpcDt);

            trajectory.col(i) << current.yaw, yaw_velocity, current.pitch, pitch_velocity;
        }

        return trajectory;
    }
};

ReferenceBuilder::ReferenceBuilder() noexcept
    : pimpl { std::make_unique<Impl>() } { }

ReferenceBuilder::~ReferenceBuilder() noexcept = default;

auto ReferenceBuilder::build(predictor::Snapshot const& snapshot,
    TargetSolution const& raw_solution, TimePoint command_time, double bullet_speed,
    double shoot_delay) -> std::expected<ReferenceTrajectory, std::string> {
    return pimpl->build(snapshot, raw_solution, command_time, bullet_speed, shoot_delay);
}

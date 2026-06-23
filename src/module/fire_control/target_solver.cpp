#include "module/fire_control/target_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <vector>

#include "module/fire_control/trajectory_solution.hpp"
#include "utility/math/angle.hpp"

using namespace rmcs::fire_control;

struct TargetSolver::Impl {
    static constexpr int kMaxIterateCount        = 5;
    static constexpr double kMaxFlyTimeThreshold = 0.001;

    struct AimSample {
        Armor3d armor;
        AimAttitude attitude;
    };

    static auto make_candidates(predictor::Snapshot const& snapshot, TimePoint sample_time)
        -> std::vector<ArmorCandidate> {
        auto candidates       = std::vector<ArmorCandidate> {};
        auto predicted_armors = snapshot.predicted_armors(sample_time);
        auto predicted_motion = snapshot.motion_at(sample_time);
        candidates.reserve(predicted_armors.size());

        for (auto const& armor : predicted_armors) {
            candidates.emplace_back(ArmorCandidate {
                .armor  = armor,
                .motion = predicted_motion,
            });
        }

        return candidates;
    }

    static auto solve(predictor::Snapshot const& snapshot, int armor_id, TimePoint command_time,
        double bullet_speed, double shoot_delay) -> std::expected<TargetSolution, std::string> {
        auto target_motion    = snapshot.motion_at(command_time);
        auto target_position  = target_motion.center_position.make<Eigen::Vector3d>();
        auto current_fly_time = target_position.norm() / bullet_speed;

        auto result = std::optional<TargetSolution> {};

        for (int i = 0; i < kMaxIterateCount; ++i) {
            auto const elapsed_command_time =
                std::chrono::duration<double>(command_time - snapshot.time_stamp()).count();
            auto const remaining_shoot_delay = std::max(0.0, shoot_delay - elapsed_command_time);
            auto const total_predict_time    = remaining_shoot_delay + current_fly_time;
            auto const t_target              = command_time
                + std::chrono::duration_cast<Duration>(
                    std::chrono::duration<double>(total_predict_time));

            auto predicted_motion = snapshot.motion_at(t_target);
            auto sample           = sample_at(snapshot, armor_id, t_target, bullet_speed);
            if (!sample.has_value()) return std::unexpected { sample.error() };

            auto const time_error = std::abs(sample->attitude.fly_time - current_fly_time);
            current_fly_time      = sample->attitude.fly_time;

            result = {
                .impact_time = t_target,
                .candidate = {
                        .armor  = sample->armor,
                        .motion = predicted_motion,
                    },
                .attitude = sample->attitude,
            };
            if (time_error < kMaxFlyTimeThreshold) break;
        }

        if (!result.has_value()) return std::unexpected { "no valid impact target" };

        return *result;
    }

private:
    static auto solve_aim_attitude(Eigen::Vector3d const& aim_point, double bullet_speed)
        -> std::expected<AimAttitude, std::string> {
        auto const target_d = std::hypot(aim_point.x(), aim_point.y());
        if (!(target_d > 0.0)) {
            return std::unexpected { "invalid target distance" };
        }

        auto solution                  = TrajectorySolution {};
        solution.input.v0              = bullet_speed;
        solution.input.target_position = aim_point;

        auto trajectory = solution.solve();
        if (!trajectory.has_value()) {
            return std::unexpected { "trajectory solve failed" };
        }

        return AimAttitude {
            .yaw      = util::normalize_angle(trajectory->yaw),
            .pitch    = -trajectory->pitch,
            .fly_time = trajectory->fly_time,
        };
    }

    static auto sample_at(predictor::Snapshot const& snapshot, int armor_id, TimePoint t,
        double bullet_speed) -> std::expected<AimSample, std::string> {
        auto predicted_armors = snapshot.predicted_armors(t);
        auto chosen_armor     = std::optional<Armor3d> {};
        for (auto const& armor : predicted_armors) {
            if (armor.id != armor_id) continue;
            chosen_armor = armor;
            break;
        }
        if (!chosen_armor.has_value()) return std::unexpected { "locked armor id not found" };

        auto aim_point = Eigen::Vector3d {};
        chosen_armor->translation.copy_to(aim_point);

        auto attitude = solve_aim_attitude(aim_point, bullet_speed);
        if (!attitude.has_value()) return std::unexpected { attitude.error() };

        return AimSample {
            .armor    = *chosen_armor,
            .attitude = *attitude,
        };
    }
};

TargetSolver::TargetSolver() noexcept
    : pimpl { std::make_unique<Impl>() } { }

TargetSolver::~TargetSolver() noexcept = default;

auto TargetSolver::make_candidates(predictor::Snapshot const& snapshot, TimePoint sample_time) const
    -> std::vector<ArmorCandidate> {
    return pimpl->make_candidates(snapshot, sample_time);
}

auto TargetSolver::solve(predictor::Snapshot const& snapshot, int armor_id, TimePoint command_time,
    double bullet_speed, double shoot_delay) const -> std::expected<TargetSolution, std::string> {
    return pimpl->solve(snapshot, armor_id, command_time, bullet_speed, shoot_delay);
}

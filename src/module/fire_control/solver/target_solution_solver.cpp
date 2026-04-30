#include "module/fire_control/solver/target_solution_solver.hpp"

#include <chrono>
#include <cmath>
#include <optional>

#include "module/fire_control/solver/aim_point_sampling.hpp"
using namespace rmcs::fire_control;

struct TargetSolutionSolver::Impl {
    static constexpr int kMaxIterateCount        = 5;
    static constexpr double kMaxFlyTimeThreshold = 0.001;

    static auto solve(predictor::Snapshot const& snapshot, AimPointChooser& chooser,
        double bullet_speed, double shoot_delay) -> std::expected<TargetSolution, std::string> {
        struct BestCandidate {
            AimAttitude attitude;
            TimePoint impact_time;
            Eigen::Vector3d center_position;
        };

        // 使用整车坐标计算飞行时间初值
        auto target_kinematics = snapshot.kinematics();
        auto target_position   = target_kinematics.center_position;
        auto current_fly_time  = target_position.norm() / bullet_speed;

        auto best_candidate = std::optional<BestCandidate> {};

        for (int i = 0; i < kMaxIterateCount; ++i) {
            auto const total_predict_time = current_fly_time + shoot_delay;
            auto const t_target           = snapshot.time_stamp()
                + std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(total_predict_time));

            auto predicted_kinematics = snapshot.kinematics_at(t_target);
            auto attitude =
                AimPointSampler::sample_attitude_at(snapshot, chooser, t_target, bullet_speed);
            if (!attitude.has_value()) continue;

            auto const time_error = std::abs(attitude->fly_time - current_fly_time);
            current_fly_time      = attitude->fly_time;
            best_candidate        = BestCandidate {
                       .attitude        = *attitude,
                       .impact_time     = t_target,
                       .center_position = predicted_kinematics.center_position,
            };
            if (time_error < kMaxFlyTimeThreshold) break;
        }

        if (!best_candidate.has_value()) {
            return std::unexpected { "no valid impact target" };
        }

        return TargetSolution {
            .impact_time     = best_candidate->impact_time,
            .impact_yaw      = best_candidate->attitude.yaw,
            .impact_pitch    = best_candidate->attitude.pitch,
            .center_position = best_candidate->center_position,
        };
    }
};

TargetSolutionSolver::TargetSolutionSolver() noexcept
    : pimpl { std::make_unique<Impl>() } { }

TargetSolutionSolver::~TargetSolutionSolver() noexcept = default;

auto TargetSolutionSolver::solve(predictor::Snapshot const& snapshot, AimPointChooser& chooser,
    double bullet_speed, double shoot_delay) const -> std::expected<TargetSolution, std::string> {
    return pimpl->solve(snapshot, chooser, bullet_speed, shoot_delay);
}

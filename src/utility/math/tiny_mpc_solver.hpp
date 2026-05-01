#pragma once

#include <expected>
#include <format>
#include <memory>
#include <string>
#include <utility>

#include <eigen3/Eigen/Geometry>
#include <tinympc/tiny_api.hpp>

namespace rmcs::util {

template <std::size_t kStateDim, std::size_t kInputDim, std::size_t kHorizonSteps>
class TinyMpcSolver {
    static_assert(kStateDim > 0, "state_dim must be positive");
    static_assert(kInputDim > 0, "input_dim must be positive");
    static_assert(kHorizonSteps >= 2, "horizon_steps must be at least 2");

private:
    using StateMatrix = Eigen::Matrix<double, kStateDim, kStateDim>;
    using InputMatrix = Eigen::Matrix<double, kStateDim, kInputDim>;
    using StateCost   = Eigen::Matrix<double, kStateDim, kStateDim>;
    using InputCost   = Eigen::Matrix<double, kInputDim, kInputDim>;

public:
    using StateVector     = Eigen::Matrix<double, kStateDim, 1>;
    using StateTrajectory = Eigen::Matrix<double, kStateDim, kHorizonSteps>;
    using InputTrajectory = Eigen::Matrix<double, kInputDim, kHorizonSteps - 1>;

    struct SetupConfig {
        StateMatrix A;
        InputMatrix B;
        StateVector f;
        StateCost Q;
        InputCost R;
        double rho { 1.0 };
        int verbose { 0 };
    };

    struct BoundConstraints {
        StateTrajectory x_min;
        StateTrajectory x_max;
        InputTrajectory u_min;
        InputTrajectory u_max;
    };

    struct InitConfig {
        SetupConfig setup;
        BoundConstraints bounds;
        int max_iter { 10 };
    };

    TinyMpcSolver(TinyMpcSolver const&)                = delete;
    TinyMpcSolver& operator=(TinyMpcSolver const&)     = delete;
    TinyMpcSolver(TinyMpcSolver&&) noexcept            = default;
    TinyMpcSolver& operator=(TinyMpcSolver&&) noexcept = default;

    static auto create(InitConfig const& config) -> std::expected<TinyMpcSolver, std::string> {
        if (config.max_iter <= 0) {
            return std::unexpected {
                std::format("max_iter must be positive, got {}", config.max_iter),
            };
        }

        TinySolver* raw_solver = nullptr;
        const auto status = tiny_setup(&raw_solver, config.setup.A, config.setup.B, config.setup.f,
            config.setup.Q, config.setup.R, config.setup.rho, kStateDim, kInputDim, kHorizonSteps,
            config.setup.verbose);
        if (status != 0 || raw_solver == nullptr) {
            TinySolverDeleter { }(raw_solver);
            return std::unexpected {
                std::format("tiny_setup failed with status {}", status),
            };
        }

        auto solver             = TinyMpcSolver { TinySolverPtr { raw_solver } };
        const auto bound_status = tiny_set_bound_constraints(solver.solver_.get(),
            config.bounds.x_min, config.bounds.x_max, config.bounds.u_min, config.bounds.u_max);
        if (bound_status != 0) {
            return std::unexpected {
                std::format("tiny_set_bound_constraints failed with status {}", bound_status),
            };
        }

        solver.solver_->settings->max_iter = config.max_iter;
        return solver;
    }

    auto set_x0(StateVector const& x0) -> std::expected<void, std::string> {
        const auto status = tiny_set_x0(solver_.get(), x0);
        if (status != 0) {
            return std::unexpected {
                std::format("tiny_set_x0 failed with status {}", status),
            };
        }

        return { };
    }

    auto set_x_ref(StateTrajectory const& x_ref) -> std::expected<void, std::string> {
        const auto status = tiny_set_x_ref(solver_.get(), x_ref);
        if (status != 0) {
            return std::unexpected {
                std::format("tiny_set_x_ref failed with status {}", status),
            };
        }

        return { };
    }

    auto set_u_ref(InputTrajectory const& u_ref) -> std::expected<void, std::string> {
        const auto status = tiny_set_u_ref(solver_.get(), u_ref);
        if (status != 0) {
            return std::unexpected {
                std::format("tiny_set_u_ref failed with status {}", status),
            };
        }

        return { };
    }

    auto solve() -> std::expected<void, std::string> {
        const auto status = tiny_solve(solver_.get());
        if (status != 0) {
            return std::unexpected {
                std::format("tiny_solve failed with status {}", status),
            };
        }

        return { };
    }

    template <std::size_t row, std::size_t step>
    auto state() const -> double {
        static_assert(row < kStateDim, "state row is out of range");
        static_assert(step < kHorizonSteps, "state step is out of range");

        return solver_->work->x(row, step);
    }

    template <std::size_t row, std::size_t step>
    auto input() const -> double {
        static_assert(row < kInputDim, "input row is out of range");
        static_assert(step < kHorizonSteps - 1, "input step is out of range");

        return solver_->work->u(row, step);
    }

private:
    struct TinySolverDeleter {
        auto operator()(TinySolver* solver) const noexcept -> void {
            if (solver == nullptr) return;

            delete solver->solution;
            delete solver->settings;
            delete solver->cache;
            delete solver->work;
            delete solver;
        }
    };

    using TinySolverPtr = std::unique_ptr<TinySolver, TinySolverDeleter>;

    explicit TinyMpcSolver(TinySolverPtr solver) noexcept
        : solver_ { std::move(solver) } { }

    TinySolverPtr solver_ { };
};

} // namespace rmcs::util

#pragma once

#include <expected>
#include <format>
#include <memory>
#include <string>
#include <utility>

#include <eigen3/Eigen/Core>

#include <tinympc/tiny_api.hpp>

namespace rmcs::util {

template <int StateDim, int InputDim, int HorizonSteps>
class TinyMpcSolver {
    static_assert(StateDim > 0, "StateDim must be positive");
    static_assert(InputDim > 0, "InputDim must be positive");
    static_assert(HorizonSteps >= 2, "HorizonSteps must be at least 2");

private:
    using StateMatrix = Eigen::Matrix<double, StateDim, StateDim>;
    using InputMatrix = Eigen::Matrix<double, StateDim, InputDim>;
    using StateCost   = Eigen::Matrix<double, StateDim, StateDim>;
    using InputCost   = Eigen::Matrix<double, InputDim, InputDim>;

public:
    using StateVector     = Eigen::Matrix<double, StateDim, 1>;
    using StateTrajectory = Eigen::Matrix<double, StateDim, HorizonSteps>;
    using InputTrajectory = Eigen::Matrix<double, InputDim, HorizonSteps - 1>;

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

    ~TinyMpcSolver() noexcept = default;

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
        auto const status = tiny_setup(&raw_solver, config.setup.A, config.setup.B, config.setup.f,
            config.setup.Q, config.setup.R, config.setup.rho, StateDim, InputDim, HorizonSteps,
            config.setup.verbose);
        if (status != 0 || raw_solver == nullptr) {
            tiny_destroy(raw_solver);
            return std::unexpected {
                std::format("tiny_setup failed with status {}", status),
            };
        }

        auto solver             = TinyMpcSolver { TinySolverPtr { raw_solver } };
        auto const bound_status = tiny_set_bound_constraints(solver.solver_.get(),
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
        auto const status = tiny_set_x0(solver_.get(), x0);
        if (status != 0) {
            return std::unexpected {
                std::format("tiny_set_x0 failed with status {}", status),
            };
        }

        return {};
    }

    auto set_x_ref(StateTrajectory const& x_ref) -> std::expected<void, std::string> {
        auto const status = tiny_set_x_ref(solver_.get(), x_ref);
        if (status != 0) {
            return std::unexpected {
                std::format("tiny_set_x_ref failed with status {}", status),
            };
        }

        return {};
    }

    auto set_u_ref(InputTrajectory const& u_ref) -> std::expected<void, std::string> {
        auto const status = tiny_set_u_ref(solver_.get(), u_ref);
        if (status != 0) {
            return std::unexpected {
                std::format("tiny_set_u_ref failed with status {}", status),
            };
        }

        return {};
    }

    auto solve() -> std::expected<void, std::string> {
        auto const status = tiny_solve(solver_.get());
        if (status != 0) {
            return std::unexpected {
                std::format("tiny_solve failed with status {}", status),
            };
        }

        return {};
    }

    auto state_value(int row, int step) const -> std::expected<double, std::string> {
        if (row < 0 || row >= StateDim) {
            return std::unexpected {
                std::format("state row {} is out of range [0, {})", row, StateDim),
            };
        }
        if (step < 0 || step >= HorizonSteps) {
            return std::unexpected {
                std::format("state step {} is out of range [0, {})", step, HorizonSteps),
            };
        }

        return solver_->work->x(row, step);
    }

    auto input_value(int row, int step) const -> std::expected<double, std::string> {
        if (row < 0 || row >= InputDim) {
            return std::unexpected {
                std::format("input row {} is out of range [0, {})", row, InputDim),
            };
        }
        if (step < 0 || step >= HorizonSteps - 1) {
            return std::unexpected {
                std::format("input step {} is out of range [0, {})", step, HorizonSteps - 1),
            };
        }

        return solver_->work->u(row, step);
    }

private:
    struct TinySolverDeleter {
        auto operator()(TinySolver* solver) const noexcept -> void { tiny_destroy(solver); }
    };

    using TinySolverPtr = std::unique_ptr<TinySolver, TinySolverDeleter>;

    explicit TinyMpcSolver(TinySolverPtr solver) noexcept
        : solver_ { std::move(solver) } { }

    TinySolverPtr solver_ {};
};

} // namespace rmcs::util

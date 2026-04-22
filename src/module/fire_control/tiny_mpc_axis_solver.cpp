#include "tiny_mpc_axis_solver.hpp"

#include <format>
#include <memory>

#include <tinympc/tiny_api.hpp>

using namespace rmcs::fire_control;

namespace {

constexpr int kAxisStateDim      = 2;
constexpr int kAxisInputDim      = 1;
constexpr double kMpcAxisDt      = 0.01;
constexpr double kSolverRho      = 1.0;
constexpr double kUnboundedState = 1e17;

using AxisState       = Eigen::Matrix<double, kAxisStateDim, 1>;
using AxisDynamics    = Eigen::Matrix<double, kAxisStateDim, kAxisStateDim>;
using AxisInputMatrix = Eigen::Matrix<double, kAxisStateDim, kAxisInputDim>;
using AxisStateCost   = Eigen::Matrix<double, kAxisStateDim, kAxisStateDim>;
using AxisInputCost   = Eigen::Matrix<double, kAxisInputDim, kAxisInputDim>;

} // namespace

struct TinyMpcAxisSolver::Impl {
    struct TinySolverDeleter {
        auto operator()(TinySolver* solver) const noexcept -> void { tiny_destroy(solver); }
    };

    using TinySolverPtr = std::unique_ptr<TinySolver, TinySolverDeleter>;

    TinySolverPtr solver {};

    auto initialize(TinyMpcAxisSolver::Config const& config) -> std::expected<void, std::string> {
        AxisDynamics A;
        A << 1.0, kMpcAxisDt, 0.0, 1.0;

        AxisInputMatrix B;
        B << 0.0, kMpcAxisDt;

        auto const f = AxisState::Zero();

        AxisStateCost Q = AxisStateCost::Zero();
        Q(0, 0)         = config.q_angle;
        Q(1, 1)         = config.q_rate;

        AxisInputCost R;
        R << config.r_acc;

        TinySolver* raw_solver  = nullptr;
        auto const setup_status = tiny_setup(&raw_solver, A, B, f, Q, R, kSolverRho, kAxisStateDim,
            kAxisInputDim, kMpcAxisHorizon, 0);
        if (setup_status != 0 || raw_solver == nullptr) {
            tiny_destroy(raw_solver);
            return std::unexpected {
                std::format("tiny_setup failed with status {}", setup_status),
            };
        }
        auto new_solver = TinySolverPtr { raw_solver };

        auto const x_min =
            Eigen::MatrixXd::Constant(kAxisStateDim, kMpcAxisHorizon, -kUnboundedState);
        auto const x_max =
            Eigen::MatrixXd::Constant(kAxisStateDim, kMpcAxisHorizon, kUnboundedState);
        auto const u_min =
            Eigen::MatrixXd::Constant(kAxisInputDim, kMpcAxisHorizon - 1, -config.max_acc);
        auto const u_max =
            Eigen::MatrixXd::Constant(kAxisInputDim, kMpcAxisHorizon - 1, config.max_acc);
        auto const bounds_status =
            tiny_set_bound_constraints(new_solver.get(), x_min, x_max, u_min, u_max);
        if (bounds_status != 0) {
            return std::unexpected {
                std::format("tiny_set_bound_constraints failed with status {}", bounds_status),
            };
        }

        new_solver->settings->max_iter = config.max_iter;
        solver                         = std::move(new_solver);
        return {};
    }

    auto solve_at_step(MpcAxisTrajectory const& reference, int step) const
        -> std::expected<double, std::string> {
        if (solver == nullptr) {
            return std::unexpected { "solver is not initialized" };
        }
        if (step < 0 || step >= kMpcAxisHorizon) {
            return std::unexpected {
                std::format("step {} is out of range [0, {})", step, kMpcAxisHorizon),
            };
        }

        AxisState x0         = reference.col(0);
        auto const x0_status = tiny_set_x0(solver.get(), x0);
        if (x0_status != 0) {
            return std::unexpected { std::format("tiny_set_x0 failed with status {}", x0_status) };
        }

        tinyMatrix x_ref      = reference;
        auto const ref_status = tiny_set_x_ref(solver.get(), x_ref);
        if (ref_status != 0) {
            return std::unexpected {
                std::format("tiny_set_x_ref failed with status {}", ref_status),
            };
        }

        auto const solve_status = tiny_solve(solver.get());
        if (solve_status != 0) {
            return std::unexpected {
                std::format("tiny_solve failed with status {}", solve_status),
            };
        }

        return solver->work->x(0, step);
    }
};

TinyMpcAxisSolver::TinyMpcAxisSolver() noexcept
    : pimpl { std::make_unique<Impl>() } { };

TinyMpcAxisSolver::~TinyMpcAxisSolver() noexcept = default;

auto TinyMpcAxisSolver::initialize(Config const& config) -> std::expected<void, std::string> {
    return pimpl->initialize(config);
}

auto TinyMpcAxisSolver::solve_center(MpcAxisTrajectory const& reference)
    -> std::expected<double, std::string> {
    return pimpl->solve_at_step(reference, kMpcAxisHorizon / 2);
}

#include "module/fire_control/planner/tiny_mpc_axis_solver.hpp"

#include <memory>
#include <optional>

#include "utility/control/tiny_mpc_solver.hpp"

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
using AxisSolver      = rmcs::util::TinyMpcSolver<kAxisStateDim, kAxisInputDim, kMpcAxisHorizon>;

} // namespace

struct TinyMpcAxisSolver::Impl {
    std::optional<AxisSolver> solver {};

    auto initialize(Config const& config) -> std::expected<void, std::string> {
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

        auto created = AxisSolver::create(AxisSolver::InitConfig {
            .setup =
                AxisSolver::SetupConfig {
                    .A   = A,
                    .B   = B,
                    .f   = f,
                    .Q   = Q,
                    .R   = R,
                    .rho = kSolverRho,
                },
            .bounds =
                AxisSolver::BoundConstraints {
                    .x_min = AxisSolver::StateTrajectory::Constant(-kUnboundedState),
                    .x_max = AxisSolver::StateTrajectory::Constant(kUnboundedState),
                    .u_min = AxisSolver::InputTrajectory::Constant(-config.max_acc),
                    .u_max = AxisSolver::InputTrajectory::Constant(config.max_acc),
                },
            .max_iter = config.max_iter,
        });
        if (!created.has_value()) {
            return std::unexpected { created.error() };
        }

        solver = std::move(created).value();
        return {};
    }

    auto solve_at_step(MpcAxisTrajectory const& reference, int step)
        -> std::expected<double, std::string> {
        if (!solver.has_value()) {
            return std::unexpected { "solver is not initialized" };
        }

        if (auto result = solver->set_x0(reference.col(0)); !result.has_value()) {
            return std::unexpected { result.error() };
        }

        if (auto result = solver->set_x_ref(reference); !result.has_value()) {
            return std::unexpected { result.error() };
        }

        if (auto result = solver->solve(); !result.has_value()) {
            return std::unexpected { result.error() };
        }

        return solver->state_value(0, step);
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

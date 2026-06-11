#include "module/fire_control/planner/mpc_solver.hpp"

#include <memory>
#include <optional>

#include "utility/math/tiny_mpc_solver.hpp"

using namespace rmcs::fire_control;

namespace {

constexpr int kAxisStateDim      = 2;
constexpr int kAxisInputDim      = 1;
constexpr double kSolverRho      = 1.0;
constexpr double kUnboundedState = 1e17;

using AxisState       = Eigen::Matrix<double, kAxisStateDim, 1>;
using AxisDynamics    = Eigen::Matrix<double, kAxisStateDim, kAxisStateDim>;
using AxisInputMatrix = Eigen::Matrix<double, kAxisStateDim, kAxisInputDim>;
using AxisStateCost   = Eigen::Matrix<double, kAxisStateDim, kAxisStateDim>;
using AxisInputCost   = Eigen::Matrix<double, kAxisInputDim, kAxisInputDim>;
using AxisSolver      = rmcs::util::TinyMpcSolver<kAxisStateDim, kAxisInputDim, kMpcHorizon>;

} // namespace

struct MpcAxisSolver::Impl {
    std::optional<AxisSolver> solver {};

    auto initialize(Config const& config) -> std::expected<void, std::string> {
        auto A = AxisDynamics {};
        A << 1.0, kMpcDt, 0.0, 1.0;

        auto B = AxisInputMatrix {};
        B << 0.0, kMpcDt;

        const auto f = AxisState::Zero();

        auto Q  = AxisStateCost { AxisStateCost::Zero() };
        Q(0, 0) = config.q_angle;
        Q(1, 1) = config.q_rate;

        auto R = AxisInputCost {};
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

    auto solve_kinematics(MpcAxisTrajectory const& reference,
        MpcAxisSolver::AngularKinematics const& initial)
        -> std::expected<MpcAxisSolver::AngularKinematics, std::string> {
        if (!solver.has_value()) {
            return std::unexpected { "solver is not initialized" };
        }

        auto x0 = AxisState {};
        x0 << initial.angle, initial.rate;
        if (auto result = solver->set_x0(x0); !result.has_value()) {
            return std::unexpected { result.error() };
        }

        if (auto result = solver->set_x_ref(reference); !result.has_value()) {
            return std::unexpected { result.error() };
        }

        if (auto result = solver->solve(); !result.has_value()) {
            return std::unexpected { result.error() };
        }

        return MpcAxisSolver::AngularKinematics {
            .angle = solver->template state<0, 1>(),
            .rate  = solver->template state<1, 1>(),
            .acc   = solver->template input<0, 0>(),
        };
    }
};

MpcAxisSolver::MpcAxisSolver() noexcept
    : pimpl { std::make_unique<Impl>() } { }

MpcAxisSolver::~MpcAxisSolver() noexcept = default;

auto MpcAxisSolver::initialize(Config const& config) -> std::expected<void, std::string> {
    return pimpl->initialize(config);
}

auto MpcAxisSolver::solve_kinematics(
    MpcAxisTrajectory const& reference, AngularKinematics const& initial)
    -> std::expected<AngularKinematics, std::string> {
    return pimpl->solve_kinematics(reference, initial);
}

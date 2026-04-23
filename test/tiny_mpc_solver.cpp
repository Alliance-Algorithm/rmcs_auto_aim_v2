#include "utility/control/tiny_mpc_solver.hpp"

#include <gtest/gtest.h>

using namespace rmcs::util;

namespace {

constexpr int kHorizon           = 100;
constexpr int kAngleStateIndex   = 0;
constexpr int kAngleInputIndex   = 0;
constexpr double kDt             = 0.01;
constexpr double kDefaultMaxAcc  = 50.0;

using Solver = TinyMpcSolver<2, 1, kHorizon>;

auto make_setup_config() -> Solver::SetupConfig {
    auto A = Eigen::Matrix2d {};
    A << 1.0, kDt, 0.0, 1.0;

    auto B = Eigen::Matrix<double, 2, 1> {};
    B << 0.0, kDt;

    Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();
    Q(0, 0) = 9e6;

    auto R = Eigen::Matrix<double, 1, 1> {};
    R << 1.0;

    return Solver::SetupConfig {
        .A   = A,
        .B   = B,
        .f   = Eigen::Vector2d::Zero(),
        .Q   = Q,
        .R   = R,
        .rho = 1.0,
    };
}

auto make_bounds(double max_acc = kDefaultMaxAcc) -> Solver::BoundConstraints {
    return Solver::BoundConstraints {
        .x_min = Solver::StateTrajectory::Constant(-1e17),
        .x_max = Solver::StateTrajectory::Constant(1e17),
        .u_min = Solver::InputTrajectory::Constant(-max_acc),
        .u_max = Solver::InputTrajectory::Constant(max_acc),
    };
}

auto make_init_config(int max_iter = 10, double max_acc = kDefaultMaxAcc) -> Solver::InitConfig {
    return Solver::InitConfig {
        .setup    = make_setup_config(),
        .bounds   = make_bounds(max_acc),
        .max_iter = max_iter,
    };
}

auto make_reference(double angle) -> Solver::StateTrajectory {
    Solver::StateTrajectory reference = Solver::StateTrajectory::Zero();
    reference.row(0).setConstant(angle);
    return reference;
}

} // namespace

TEST(tiny_mpc_solver, solves_constant_reference) {
    auto created = Solver::create(make_init_config());
    ASSERT_TRUE(created.has_value()) << created.error();
    auto solver = std::move(created).value();

    auto const reference = make_reference(0.75);
    Solver::StateVector x0 = reference.col(0);
    auto const u_ref       = Solver::InputTrajectory::Zero();

    auto result = solver.set_x0(x0);
    ASSERT_TRUE(result.has_value()) << result.error();

    result = solver.set_x_ref(reference);
    ASSERT_TRUE(result.has_value()) << result.error();

    result = solver.set_u_ref(u_ref);
    ASSERT_TRUE(result.has_value()) << result.error();

    result = solver.solve();
    ASSERT_TRUE(result.has_value()) << result.error();

    auto state = solver.state_value(kAngleStateIndex, kHorizon / 2);
    ASSERT_TRUE(state.has_value()) << state.error();
    EXPECT_NEAR(state.value(), 0.75, 1e-6);
}

TEST(tiny_mpc_solver, create_requires_positive_max_iter) {
    auto created = Solver::create(make_init_config(0));
    ASSERT_FALSE(created.has_value());
    EXPECT_EQ(created.error(), "max_iter must be positive, got 0");
}

TEST(tiny_mpc_solver, state_value_rejects_out_of_range_step) {
    auto created = Solver::create(make_init_config());
    ASSERT_TRUE(created.has_value()) << created.error();
    auto solver = std::move(created).value();

    auto state = solver.state_value(kAngleStateIndex, kHorizon);
    ASSERT_FALSE(state.has_value());
    EXPECT_EQ(state.error(), "state step 100 is out of range [0, 100)");
}

TEST(tiny_mpc_solver, input_value_rejects_out_of_range_step) {
    auto created = Solver::create(make_init_config());
    ASSERT_TRUE(created.has_value()) << created.error();
    auto solver = std::move(created).value();

    auto input = solver.input_value(kAngleInputIndex, kHorizon - 1);
    ASSERT_FALSE(input.has_value());
    EXPECT_EQ(input.error(), "input step 99 is out of range [0, 99)");
}

TEST(tiny_mpc_solver, input_value_returns_zero_for_constant_reference) {
    auto created = Solver::create(make_init_config());
    ASSERT_TRUE(created.has_value()) << created.error();
    auto solver = std::move(created).value();

    auto const reference = make_reference(0.75);
    auto const u_ref     = Solver::InputTrajectory::Zero();

    auto result = solver.set_x0(reference.col(0));
    ASSERT_TRUE(result.has_value()) << result.error();

    result = solver.set_x_ref(reference);
    ASSERT_TRUE(result.has_value()) << result.error();

    result = solver.set_u_ref(u_ref);
    ASSERT_TRUE(result.has_value()) << result.error();

    result = solver.solve();
    ASSERT_TRUE(result.has_value()) << result.error();

    auto input = solver.input_value(kAngleInputIndex, kHorizon / 2 - 1);
    ASSERT_TRUE(input.has_value()) << input.error();
    EXPECT_NEAR(input.value(), 0.0, 1e-9);
}

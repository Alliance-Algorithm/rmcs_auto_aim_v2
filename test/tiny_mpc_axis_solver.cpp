#include "module/fire_control/planner/tiny_mpc_axis_solver.hpp"

#include <gtest/gtest.h>

using namespace rmcs::fire_control;

namespace {

auto make_constant_reference(double angle) -> MpcAxisTrajectory {
    auto reference = MpcAxisTrajectory {};
    reference.setZero();
    reference.row(0).setConstant(angle);
    return reference;
}

auto make_config() -> TinyMpcAxisSolver::Config {
    return TinyMpcAxisSolver::Config {
        .max_acc  = 50.0,
        .q_angle  = 9e6,
        .q_rate   = 0.0,
        .r_acc    = 1.0,
        .max_iter = 10,
    };
}

} // namespace

TEST(tiny_mpc_axis_solver, solves_constant_reference) {
    auto solver = TinyMpcAxisSolver {};

    auto init = solver.initialize(make_config());
    ASSERT_TRUE(init.has_value()) << init.error();

    auto result = solver.solve_center(make_constant_reference(0.75));
    ASSERT_TRUE(result.has_value()) << result.error();
    EXPECT_NEAR(result.value(), 0.75, 1e-6);
}

TEST(tiny_mpc_axis_solver, solve_center_requires_initialization) {
    auto solver = TinyMpcAxisSolver {};

    auto result = solver.solve_center(make_constant_reference(0.25));
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), "solver is not initialized");
}

TEST(tiny_mpc_axis_solver, can_reinitialize_without_reset) {
    auto solver = TinyMpcAxisSolver {};

    auto init = solver.initialize(make_config());
    ASSERT_TRUE(init.has_value()) << init.error();

    init = solver.initialize(make_config());
    ASSERT_TRUE(init.has_value()) << init.error();

    auto result = solver.solve_center(make_constant_reference(0.5));
    ASSERT_TRUE(result.has_value()) << result.error();
    EXPECT_NEAR(result.value(), 0.5, 1e-6);
}

TEST(tiny_mpc_axis_solver, initialize_rejects_non_positive_max_iter) {
    auto solver = TinyMpcAxisSolver {};

    auto config     = make_config();
    config.max_iter = 0;

    auto init = solver.initialize(config);
    ASSERT_FALSE(init.has_value());
    EXPECT_EQ(init.error(), "max_iter must be positive, got 0");
}

TEST(tiny_mpc_axis_solver, can_solve_multiple_references_after_initialization) {
    auto solver = TinyMpcAxisSolver {};

    auto init = solver.initialize(make_config());
    ASSERT_TRUE(init.has_value()) << init.error();

    auto first = solver.solve_center(make_constant_reference(0.25));
    ASSERT_TRUE(first.has_value()) << first.error();
    EXPECT_NEAR(first.value(), 0.25, 1e-6);

    auto second = solver.solve_center(make_constant_reference(-0.5));
    ASSERT_TRUE(second.has_value()) << second.error();
    EXPECT_NEAR(second.value(), -0.5, 1e-6);
}

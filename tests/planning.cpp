#include "catch2/internal/catch_clara.hpp"
#define CATCH_CONFIG_MAIN

#include <Eigen/Geometry>
#include <stdexcept>

#include "catch2/catch_all.hpp"
#include "catch2/catch_test_macros.hpp"
#include "planner_rrt.hpp"

TEST_CASE("Test constructors", "[Planner]") {
  auto collision_func = [](Eigen::VectorXd) { return false; };

  SECTION("Default constructor") {
    // Basic construction - should succeed.
    REQUIRE_NOTHROW(planning::RRT{2, 10, collision_func});

    // Invalid dimension - should throw.
    REQUIRE_THROWS(planning::RRT{0, 100, collision_func});

    // Test invalid min/max values
    Eigen::VectorXd min_vals(2);
    Eigen::VectorXd max_vals(2);
    min_vals << 0, 10;
    max_vals << -1, 20;
    REQUIRE_THROWS(planning::RRT{2, min_vals, max_vals, collision_func});
  }

  SECTION("Setters") {
    planning::RRT rrt{2, 10, collision_func};
    REQUIRE_NOTHROW(rrt.SetMaxIters(100));
    REQUIRE_THROWS_AS(rrt.SetMaxIters(-10), std::invalid_argument);

    Eigen::VectorXd start_valid(2);
    Eigen::VectorXd goal_valid(2);
    start_valid << 0, 0;
    goal_valid << 10, 10;
    REQUIRE_NOTHROW(rrt.SetProblem(start_valid, goal_valid));

    Eigen::VectorXd start_invalid(3);
    Eigen::VectorXd goal_invalid(3);
    REQUIRE_THROWS_AS(rrt.SetProblem(start_invalid, goal_valid),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(rrt.SetProblem(start_valid, goal_invalid),
                      std::invalid_argument);
  }

  // TODO: Add full test coverage for all constructors
}

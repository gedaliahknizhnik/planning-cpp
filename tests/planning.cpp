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
    REQUIRE_NOTHROW(
        planning::RRT{2, 100, [](Eigen::VectorXd) { return false; }});
    REQUIRE_THROWS(planning::RRT{0, 100, collision_func});
    REQUIRE_THROWS(planning::RRT{2, 0, collision_func});
  }

  // TODO: Add full test coverage for all constructors
}

#include "planner_rrt.hpp"

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <functional>
#include <optional>
#include <stdexcept>
#include <thread>

namespace planning {

// Primary constructor - all other constructors should delegate to this one
RRT::RRT(const int dim, const Eigen::VectorXd min_vals,
         const Eigen::VectorXd max_vals, CollisionFunc collision_func)
    : dim_{dim},
      min_vals_{min_vals},
      max_vals_{max_vals},
      collision_func_{collision_func},
      logger_{logging::LogLevel::DEBUG} {
  // Dimension must be positive
  if (dim <= 0) {
    throw std::invalid_argument("Dimension must be greater than 0.");
  }
  // Check that all min values are less than the max values
  if ((min_vals.array() >= max_vals.array()).any()) {
    throw std::invalid_argument(
        "All min values must be less than their corresponding max value.");
  }

  logger_.Log(logging::LogLevel::DEBUG,
              "Created RRT Planner of dimension " + std::to_string(dim));
}
RRT::RRT(const int dim, const Eigen::VectorXd max_vals,
         CollisionFunc collision_func)
    : RRT(dim, -max_vals, max_vals, collision_func) {}
RRT::RRT(const int dim, const double max_val, CollisionFunc collision_func)
    : RRT(dim, -max_val * Eigen::VectorXd::Ones(dim),
          max_val * Eigen::VectorXd::Ones(dim), collision_func) {}

RRT::~RRT() {
  if (solver_thread_.joinable()) {
    logger_.Log(logging::LogLevel::DEBUG,
                "Waiting for solver thread to join...");
    solver_thread_.join();
  }
  logger_.Log(logging::LogLevel::DEBUG, "RRT Planner destroyed.");
}

void RRT::SetMaxIters(const int max_iters) {
  if (max_iters <= 0) {
    throw std::invalid_argument("Max iters must be greater than 0.");
  }
  max_iters_ = max_iters;
}

const PlanningStatus RRT::SetProblem(Eigen::VectorXd start,
                                     Eigen::VectorXd goal) {
  if ((start.size() != dim_) || (goal.size() != dim_)) {
    throw std::invalid_argument(
        "Start and goal dimensions must match the planner dimension.");
  }

  if (collision_func_(start)) {
    return PlanningStatus::FAILURE_START;
  } else if (collision_func_(goal)) {
    return PlanningStatus::FAILURE_GOAL;
  }

  start_ = start;
  goal_ = goal;

  plan_.clear();

  // TODO: SOLVE THE PROBLEM
  return PlanningStatus::SUCCESS;
}

const PlanningStatus RRT::SetProblem(std::vector<double> start,
                                     std::vector<double> goal) {
  return SetProblem(Eigen::Map<Eigen::VectorXd>(start.data(), start.size()),
                    Eigen::Map<Eigen::VectorXd>(goal.data(), goal.size()));
}

const std::optional<Plan> RRT::GetPlan() const {
  if (is_solved_) {
    return plan_;
  } else {
    return {};
  }
}

void RRT::Solve() {
  // Spin up a thread to solve the problem asynchronously
  solver_thread_ = std::thread{&RRT::SolveInThread, this};
}

void RRT::SolveInThread() {
  is_solved_ = false;

  for (size_t iter{0UL}; iter < max_iters_; ++iter) {
  }

  // TODO: SOLVE THE PROBLEM IN A THREAD
  // std::this_thread::sleep_for(std::chrono::seconds(10));
  is_solved_ = true;
}

}  // namespace planning

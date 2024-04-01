#include "planner_rrt.hpp"

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <functional>
#include <optional>
#include <thread>

namespace planning {

RRT::RRT(const int dim, const int max_iters, CollisionFunc collision_func)
    : dim_(dim), max_iters_{max_iters}, collision_func_{collision_func} {};
RRT::~RRT() {
  if (solver_thread_.joinable()) {
    solver_thread_.join();
  }
}

const PlanningStatus RRT::SetProblem(Eigen::VectorXd start,
                                     Eigen::VectorXd goal) {
  if ((start.size() != dim_) || (goal.size() != dim_)) {
    throw std::runtime_error(
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
  // TODO: SOLVE THE PROBLEM IN A THREAD
  // std::this_thread::sleep_for(std::chrono::seconds(10));
  is_solved_ = true;
}

}  // namespace planning

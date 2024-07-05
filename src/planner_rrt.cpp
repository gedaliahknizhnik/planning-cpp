#include "planner_rrt.hpp"

#include <Eigen/src/Core/Matrix.h>

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <optional>
#include <queue>
#include <random>
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

  logger_.Log(logging::LogLevel::WARNING,
              "Initializing random number generator using constant seed. "
              "Disable before use.");
  srand(0);
  // srand(rd_());
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
  tree_ = Tree{};

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

const std::optional<Tree> RRT::GetTree() const {
  if (is_solved_) {
    return tree_;
  } else {
    return {};
  }
}

Eigen::VectorXd RRT::RandomSample() const {
  const Eigen::VectorXd random{Eigen::VectorXd::Random(dim_)};
  const Eigen::VectorXd sample{min_vals_ +
                               (max_vals_ - min_vals_).cwiseProduct(random)};
  return sample;
}

void RRT::Solve() {
  // Spin up a thread to solve the problem asynchronously
  solver_thread_ = std::thread{&RRT::SolveInThread, this};
}

void RRT::SolveInThread() {
  is_solved_ = false;

  Eigen::MatrixXd tree;
  tree.resize(max_iters_, dim_);
  Eigen::VectorXd parent;
  parent.resize(max_iters_);

  // Add start to the queue
  size_t iter{0UL};
  tree.row(iter) = start_;
  parent(iter) = -1;
  ++iter;

  // Process RRT Iterations
  while (iter < max_iters_) {
    // Generate a random sample
    // TODO: Sometimes select the goal
    const Eigen::VectorXd sample{RandomSample()};
    if (collision_func_(sample)) {
      continue;
    }

    // Find the nearest point already in the tree
    const Eigen::MatrixXd diff_matrix =
        tree(Eigen::seq(0, iter), Eigen::all).rowwise() - sample.transpose();
    const Eigen::VectorXd diff_norms =
        diff_matrix.rowwise().norm();  // get location of minimum
    Eigen::Index min_row, min_col;
    const double min = diff_norms.minCoeff(&min_row, &min_col);
    const Eigen::VectorXd nearest = tree.row(min_row);

    // Move towards the sample
    const double distance = (sample - nearest).norm();
    const Eigen::VectorXd direction = (sample - nearest).normalized();
    const Eigen::VectorXd new_point =
        nearest + direction * std::min(distance, step_size_);

    // Add the new point to the tree
    tree.row(iter) = new_point;
    parent(iter) = min_row;

    ++iter;
  }

  tree_ = Tree{tree, parent};
  std::cout << tree << std::endl;

  // TODO: SOLVE THE PROBLEM IN A THREAD
  // std::this_thread::sleep_for(std::chrono::seconds(10));
  is_solved_ = true;
}

}  // namespace planning

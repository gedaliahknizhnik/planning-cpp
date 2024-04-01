
#include <Eigen/Geometry>
#include <atomic>
#include <functional>
#include <thread>

namespace planning {

using CollisionFunc = std::function<bool(Eigen::VectorXd)>;
using Plan = std::vector<Eigen::VectorXd>;

enum class PlanningStatus : uint {
  SUCCESS = 0U,
  FAILURE = 1U,
  FAILURE_START = 2U,
  FAILURE_GOAL = 3U,
  FAILURE_TIMEOUT = 4U,
};

class RRT {
 public:
  RRT(const int dim, const int max_iters, CollisionFunc collision_func)
      : dim_(dim), max_iters_{max_iters}, collision_func_{collision_func} {};
  ~RRT() {
    if (solver_thread_.joinable()) {
      solver_thread_.join();
    }
  }

  const PlanningStatus SetProblem(Eigen::VectorXd start, Eigen::VectorXd goal) {
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

  const PlanningStatus SetProblem(std::vector<double> start,
                                  std::vector<double> goal) {
    return SetProblem(Eigen::Map<Eigen::VectorXd>(start.data(), start.size()),
                      Eigen::Map<Eigen::VectorXd>(goal.data(), goal.size()));
  }

  const Plan GetPlan() const {
    if (is_solved_) {
      return plan_;
    } else {
      return {};
    }
  }

  void Solve() {
    // Spin up a thread to solve the problem asynchronously
    solver_thread_ = std::thread{&RRT::SolveInThread, this};
  }

 private:
  void SolveInThread() {
    is_solved_ = false;
    // TODO: SOLVE THE PROBLEM IN A THREAD
    is_solved_ = true;
  }

  const int dim_{};
  const int max_iters_{};

  CollisionFunc collision_func_{};
  std::thread solver_thread_{};
  std::atomic<bool> is_solved_{false};

  Eigen::VectorXd start_{};
  Eigen::VectorXd goal_{};
  Plan plan_{};
};

}  // namespace planning

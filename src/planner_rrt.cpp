
#include <Eigen/Geometry>
#include <functional>

namespace planning {

using CollisionFunc = std::function<bool(Eigen::VectorXd)>;

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
  ~RRT() = default;

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

    // TODO: SOLVE THE PROBLEM
    return PlanningStatus::SUCCESS;
  }

  const PlanningStatus SetProblem(std::vector<double> start,
                                  std::vector<double> goal) {
    return SetProblem(Eigen::Map<Eigen::VectorXd>(start.data(), start.size()),
                      Eigen::Map<Eigen::VectorXd>(goal.data(), goal.size()));
  }

 private:
  const int dim_{};
  const int max_iters_{};

  CollisionFunc collision_func_{};

  Eigen::VectorXd start_{};
  Eigen::VectorXd goal_{};
};
}  // namespace planning

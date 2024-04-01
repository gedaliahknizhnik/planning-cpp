#ifndef PLANNER_RRT_HPP
#define PLANNER_RRT_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <functional>
#include <optional>
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
  RRT(const int dim, const int max_iters, CollisionFunc collision_func);
  ~RRT();

  const PlanningStatus SetProblem(Eigen::VectorXd start, Eigen::VectorXd goal);
  const PlanningStatus SetProblem(std::vector<double> start,
                                  std::vector<double> goal);
  const std::optional<Plan> GetPlan() const;

  void Solve();

 private:
  void SolveInThread();

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
#endif

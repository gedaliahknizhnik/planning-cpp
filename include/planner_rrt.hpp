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

constexpr size_t MAX_ITERS_DEFAULT =
    10UL;  // TODO: Change to higher once done testing

enum class PlanningStatus : uint {
  SUCCESS = 0U,
  FAILURE = 1U,
  FAILURE_START = 2U,
  FAILURE_GOAL = 3U,
  FAILURE_TIMEOUT = 4U,
};

class RRT {
 public:
  // CONSTRUCTORS *************************************************************

  /**
   * @brief Construct a new RRT object with full specification
   *
   * Note that this is the PRIMARY constructor, and all other constructors
   * should delegate to this one.
   *
   *
   * @param dim - [int] Dimension of the problem space
   * @param min_vals - [Eigen::VectorXd] Minimum values for each dimension
   * @param max_vals - [Eigen::VectorXd] Maximum values for each dimension
   * @param collision_func - [CollisionFunc] Function to check for collision
   * @return RRT
   */
  RRT(const int dim, const Eigen::VectorXd min_vals,
      const Eigen::VectorXd max_vals, CollisionFunc collision_func);
  /**
   * @brief Construct a new RRT object with a symmetric and equal range for all
   * dimensions.
   *
   * Ranges for ALL dimensions will be [-max_val, max_val]
   *
   * @param dim - [int] Dimension of the problem space
   * @param max_vals - [Eigen::VectorXd] Maximum values for each dimension
   * @param collision_func - [CollisionFunc] Function to check for collision
   * @return RRT
   */
  RRT(const int dim, const double max_val, CollisionFunc collision_func);
  /**
   * @brief Construct a new RRT object with a symmetric range for all
   * dimensions.
   *
   * Range for EACH dimension will be [-max_vals[i], max_vals[i]]
   *
   * @param dim - [int] Dimension of the problem space
   * @param max_vals - [Eigen::VectorXd] Maximum values for each dimension
   * @param collision_func - [CollisionFunc] Function to check for collision
   * @return RRT
   */
  RRT(const int dim, const Eigen::VectorXd max_vals,
      CollisionFunc collision_func);
  /**
   * @brief Destroy the RRT object, ensuring that the solver thread is joined.
   *
   */
  ~RRT();

  void SetMaxIters(const int max_iters);
  const PlanningStatus SetProblem(Eigen::VectorXd start, Eigen::VectorXd goal);
  const PlanningStatus SetProblem(std::vector<double> start,
                                  std::vector<double> goal);
  const std::optional<Plan> GetPlan() const;

  void Solve();

 private:
  void SolveInThread();

  const int dim_{};
  int max_iters_{MAX_ITERS_DEFAULT};

  CollisionFunc collision_func_{};
  std::thread solver_thread_{};
  std::atomic<bool> is_solved_{false};

  const Eigen::VectorXd min_vals_{};
  const Eigen::VectorXd max_vals_{};

  Eigen::VectorXd start_{};
  Eigen::VectorXd goal_{};
  Plan plan_{};
};

}  // namespace planning
#endif

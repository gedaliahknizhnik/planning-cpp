
#include <Eigen/Geometry>
#include <cmath>
#include <random>

#include "planner_plotter.hpp"
#include "planner_rrt.hpp"

int main() {
  std::vector<double> start = {0, 0};
  std::vector<double> goal = {10, 11};

  planning::Plotter plt{1200, 1200, "ex.png"};
  plt.PlotStart(start);
  plt.PlotGoal(goal);

  std::vector<std::vector<double>> path = {start, {1, 1}, {1, 3}, {5, 8}, goal};
  plt.PlotPath(path);
}

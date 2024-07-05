
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
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

  planning::RRT planner{2, 100, [](Eigen::VectorXd) { return false; }};
  planner.SetProblem(start, goal);
  planner.Solve();
  while (!planner.IsSolved()) {
    // Wait for the planner to finish
    std::cout << "Waiting 10 ms for planner to finish...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::optional<planning::Tree> tree = planner.GetTree();
  if (tree.has_value()) {
    plt.PlotTree(tree.value());
    std::cout << "Plotted tree! of size: " << tree.value().GetSize()
              << std::endl;

  } else {
    std::cout << "Tree is empty!" << std::endl;
  }
}

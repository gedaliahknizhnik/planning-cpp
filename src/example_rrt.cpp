
#include <cmath>
#include <random>

#include <Eigen/Geometry>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

#include "planner_rrt.hpp"

int main() {
  std::vector<double> start = {0, 0};
  std::vector<double> goal = {10, 11};

  // Set the size of output image to 1200x780 pixels
  plt::figure_size(1200, 780);
  // Plot line from given x and y data. Color is selected automatically.
  // plt::scatter(static_cast<double>(start(0)), static_cast<double>(start(1)));
  plt::named_plot("Start", start, goal, "ro");

  // plt::named_plot("Goal", goal(0), goal(1), "go");

  // plt::xlim(0, n); Add graph title
  plt::title("Filtering Example");

  // Enable legend.
  plt::legend();
  // Save the image (file format is determined by the extension)
  plt::save("./basic.png");
}

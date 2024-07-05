#include "planner_plotter.hpp"

#include <Eigen/src/Core/Matrix.h>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

namespace planning {
Plotter::Plotter(const int dim_x, const int dim_y, const std::string& filename)
    : dim_x_(dim_x), dim_y_(dim_y), filename_(filename) {
  plt::figure_size(dim_x, dim_y);
}

Plotter::~Plotter() { Save(); }

void Plotter::PlotPoint(const std::vector<double>& point,
                        const std::string& name, const std::string& color) {
  if (point.size() != 2) {
    throw std::runtime_error("Plotter is only configured for 2D systems.");
  }

  std::vector<double> x, y;
  x.push_back(point[0]);
  y.push_back(point[1]);
  plt::named_plot(name, x, y, color);

}  // helper function

void Plotter::PlotStart(const std::vector<double>& start) {
  PlotPoint(start, "Start", "ro");
}
void Plotter::PlotGoal(const std::vector<double>& goal) {
  PlotPoint(goal, "Goal", "go");
}

void Plotter::PlotPath(const std::vector<std::vector<double>>& path) {
  std::vector<double> x, y;
  for (const auto& point : path) {
    if (point.size() != 2) {
      throw std::runtime_error("Plotter is only configured for 2D systems.");
    }

    x.push_back(point[0]);
    y.push_back(point[1]);
  }
  plt::plot(x, y, "b");
}

void Plotter::PlotLine(const Eigen::VectorXd& pti, const Eigen::VectorXd& ptj) {
  if (pti.size() != 2 || ptj.size() != 2) {
    throw std::runtime_error("Plotter is only configured for 2D systems.");
  }
  std::vector<double> x, y;
  x.push_back(pti[0]);
  x.push_back(ptj[0]);
  y.push_back(pti[1]);
  y.push_back(ptj[1]);
  plt::plot(x, y, "k");
}

void Plotter::PlotTree(const Tree& tree) {
  for (size_t ii{0UL}; ii < tree.GetSize(); ++ii) {
    Eigen::VectorXd pti, ptj;
    if (tree.GetEdge(ii, &pti, &ptj)) {
      continue;
    }
    PlotLine(pti, ptj);
  }
}

void Plotter::Save() {
  plt::legend();
  plt::save(filename_);
}

}  // namespace planning

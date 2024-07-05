#ifndef PLANNER_PLOTTER_HPP
#define PLANNER_PLOTTER_HPP

#include <Eigen/Geometry>  // Eigen is a header-only library
#include <string>
#include <vector>

#include "planner_defs.hpp"

namespace planning {

class Plotter {
 public:
  Plotter(const int dim_x, const int dim_y, const std::string& filename);
  ~Plotter();

  void PlotStart(const std::vector<double>& start);
  void PlotGoal(const std::vector<double>& goal);
  void PlotPath(const std::vector<std::vector<double>>& path);
  void PlotLine(const Eigen::VectorXd& pti, const Eigen::VectorXd& ptj);
  void PlotTree(const Tree& tree);
  void Save();

 private:
  void PlotPoint(const std::vector<double>& point, const std::string& name,
                 const std::string& color);  // helper function

  const int dim_x_{};
  const int dim_y_{};
  const std::string filename_;
};

}  // namespace planning
#endif

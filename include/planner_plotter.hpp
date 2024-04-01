#ifndef PLANNER_PLOTTER_HPP
#define PLANNER_PLOTTER_HPP

#include <string>
#include <vector>

namespace planning {

class Plotter {
 public:
  Plotter(const int dim_x, const int dim_y, const std::string& filename);
  ~Plotter();

  void PlotStart(const std::vector<double>& start);
  void PlotGoal(const std::vector<double>& goal);
  void PlotPath(const std::vector<std::vector<double>>& path);
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

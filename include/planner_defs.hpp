#ifndef PLANNER_DEFS_HPP
#define PLANNER_DEFS_HPP

#include <Eigen/Geometry>

namespace planning {

struct Tree {
  Eigen::MatrixXd nodes;
  Eigen::VectorXd parents;

  int GetEdge(const size_t ii, Eigen::VectorXd* pti,
              Eigen::VectorXd* ptj) const {
    if (ii >= nodes.rows() || ii < 0) {
      return -1;
    }
    *pti = nodes.row(ii);
    if (parents(ii) == -1) {
      return -2;
    }
    *ptj = nodes.row(parents(ii));
    return 0;
  }

  size_t GetSize() const { return nodes.rows(); }
};

}  // namespace planning

#endif

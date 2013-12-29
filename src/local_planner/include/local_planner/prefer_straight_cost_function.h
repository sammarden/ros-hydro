
#ifndef PREFER_STRAIGHT_COST_FUNCTION_H_
#define PREFER_STRAIGHT_COST_FUNCTION_H_

#include <cmath>

#include <base_local_planner/trajectory_cost_function.h>

namespace local_planner
{

class PreferStraightCostFunction : public base_local_planner::TrajectoryCostFunction
{

public:

  PreferStraightCostFunction();

  ~PreferStraightCostFunction();

  bool prepare() { return true; }

  double scoreTrajectory(base_local_planner::Trajectory & traj);

};

} // namespace local_planner

#endif // #endif PREFER_STRAIGHT_COST_FUNCTION_H_


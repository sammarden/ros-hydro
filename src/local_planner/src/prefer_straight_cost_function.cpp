
#include <local_planner/prefer_straight_cost_function.h>

namespace local_planner
{

PreferStraightCostFunction::PreferStraightCostFunction()
{

}

PreferStraightCostFunction::~PreferStraightCostFunction()
{

}

double PreferStraightCostFunction::scoreTrajectory(base_local_planner::Trajectory & traj)
{

  double score = 0.0;
  double prev_th;

  for (int i = 0; i < traj.getPointsSize(); ++i)
  {

    double x, y, th;
    traj.getPoint(i, x, y, th);

    if (i == 0)
    {
      prev_th = th;
      continue;
    }

    score += fabs(th - prev_th);

    prev_th = th;

  }

}

} // namespace local_planner


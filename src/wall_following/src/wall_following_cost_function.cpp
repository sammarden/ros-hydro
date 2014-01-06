
#include <wall_following/wall_following_cost_function.h>

namespace wall_following
{

WallFollowingCostFunction::WallFollowingCostFunction(costmap_2d::Costmap2D * costmap) : costmap_(costmap), target_dist_to_wall_(1.0), max_raytrace_dist_(2.0), wall_following_side_(LEFT)
{

}

WallFollowingCostFunction::~WallFollowingCostFunction()
{

}

bool WallFollowingCostFunction::prepare()
{
  return true;
}

double WallFollowingCostFunction::scoreTrajectory(base_local_planner::Trajectory & traj)
{

//  double cost = 0.0;
  double cost = std::numeric_limits<double>::max();

  int num_valid_points = 0;

  // For each point on the trajectory, calculate the distance to the wall by raytracing through the costmap perpendicular to the robot.
  for (unsigned int i = 0; i < traj.getPointsSize(); ++i)
  {

    double traj_x, traj_y, traj_th;
    traj.getPoint(i, traj_x, traj_y, traj_th);

    double dist_to_wall = calculateDistanceToWall(traj_x, traj_y, traj_th);

    // Only include valid distances.
    if (dist_to_wall > 0.0)
    {
//      cost += scoreDistanceToWall(dist_to_wall);
      cost = std::min(cost, scoreDistanceToWall(dist_to_wall));
      ++num_valid_points;
    }

  }

  // Return the average of valid distances to the wall.
  if (num_valid_points > 0)
  {
    return cost/(double)num_valid_points;
  }
  else
  {
    return std::numeric_limits<double>::max();
  }

}

double WallFollowingCostFunction::calculateDistanceToWall(double traj_x, double traj_y, double traj_th)
{

  double raytrace_angle;

  // Raytrace to the left of the robot.
  if (wall_following_side_ == LEFT)
  {
    raytrace_angle = traj_th + M_PI_2;
  }
  // Raytrace to the right of the robot.
  else if (wall_following_side_ == RIGHT)
  {
    raytrace_angle = traj_th - M_PI_2;
  }
  // Should never happen.
  else
  {
    ROS_ERROR("Wall following side not set");
    return -1.0;
  }

  double raytrace_end_x = traj_x + max_raytrace_dist_*cos(raytrace_angle);
  double raytrace_end_y = traj_y + max_raytrace_dist_*sin(raytrace_angle); 

  unsigned int start_cell_x, start_cell_y, end_cell_x, end_cell_y;

  // Check if we ray trace outside the costmap's bounds.
  if (!(costmap_->worldToMap(traj_x, traj_y, start_cell_x, start_cell_y) &&
        costmap_->worldToMap(raytrace_end_x, raytrace_end_y, end_cell_x, end_cell_y)))
  {
    ROS_ERROR("Raytracing outside the local costmap, max_raytrace_dist may be too large");
    return -2.0; 
  }

  // Raytrace to find the distance to the wall.
  WallFollowingRayTracer wall_following_ray_tracer(costmap_->getCharMap(), costmap_2d::LETHAL_OBSTACLE, costmap_->getSizeInCellsX());
  raytraceLine(wall_following_ray_tracer, start_cell_x, start_cell_y, end_cell_x, end_cell_y);

  // Check if we actually hit the wall.
  if (wall_following_ray_tracer.didHitWall())
  {
    double dist_to_wall = costmapIndexToDistance(wall_following_ray_tracer.getWallIndex(), traj_x, traj_y);
    return dist_to_wall;
  }
  else
  {
    return -3.0;
  }

}

double WallFollowingCostFunction::costmapIndexToDistance(unsigned int costmap_index, double traj_x, double traj_y)
{

  // Convert index to cells.
  unsigned int cell_x, cell_y;
  costmap_->indexToCells(costmap_index, cell_x, cell_y);

  // Convert cells to points.
  double wall_x, wall_y;
  costmap_->mapToWorld(cell_x, cell_y, wall_x, wall_y);

  double dx = wall_x - traj_x;
  double dy = wall_y - traj_y;

  return sqrt(dx*dx + dy*dy);

}

double WallFollowingCostFunction::scoreDistanceToWall(double dist)
{
  
  double delta_dist = fabs(dist - target_dist_to_wall_);
//  double scaled_delta_dist = delta_dist/target_dist_to_wall_;
//  return 1.01 - exp(-(scaled_delta_dist*scaled_delta_dist));
  return delta_dist/max_raytrace_dist_;
}
/**
 * @brief  Raytrace a line and apply some action at each step
 * @param  at The action to take... a functor
 * @param  x0 The starting x coordinate
 * @param  y0 The starting y coordinate
 * @param  x1 The ending x coordinate
 * @param  y1 The ending y coordinate
 * @param  max_length The maximum desired length of the segment... allows you to not go all the way to the endpoint
 */
void WallFollowingCostFunction::raytraceLine(WallFollowingRayTracer & rt, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int max_length)
  {
    int dx = x1 - x0;
    int dy = y1 - y0;

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);

    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * rt.getSizeX()/*size_x_*/;

    unsigned int offset = y0 * rt.getSizeX()/*size_x_*/ + x0;

    //we need to chose how much to scale our dominant dimension, based on the maximum length of the line
    double dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    double scale = std::min(1.0, max_length / dist);

    //if x is dominant
    if (abs_dx >= abs_dy)
    {
      int error_y = abs_dx / 2;
      bresenham2D(rt, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
      return;
    }

    //otherwise y is dominant
    int error_x = abs_dy / 2;
    bresenham2D(rt, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));

  }

/**
 * @brief  A 2D implementation of Bresenham's raytracing algorithm... applies an action at each step
 */
void WallFollowingCostFunction::bresenham2D(WallFollowingRayTracer & rt, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b, unsigned int offset, unsigned int max_length)
  {
    unsigned int end = std::min(max_length, abs_da);
    for (unsigned int i = 0; i < end; ++i)
    {
      rt(offset);
      offset += offset_a;
      error_b += abs_db;
      if ((unsigned int)error_b >= abs_da)
      {
        offset += offset_b;
        error_b -= abs_da;
      }
    }
    rt(offset);
  }


} // namespace wall_following


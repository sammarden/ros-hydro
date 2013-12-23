
#ifndef WALL_FOLLOWING_COST_FUNCTION_H_
#define WALL_FOLLOWING_COST_FUNCTION_H_

#include <ros/console.h>

#include <base_local_planner/trajectory_cost_function.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

namespace wall_following
{

enum WallFollowingSide
{
  LEFT,
  RIGHT
};

class WallFollowingRayTracer
{

public:

  WallFollowingRayTracer(unsigned char * costmap, unsigned char lethal_val, int size_x) : costmap_(costmap), lethal_val_(lethal_val), hit_wall_(false), size_x_(size_x)
  {

  }

  inline void operator()(unsigned int index)
  {

    if (hit_wall_) { return; }

    if (costmap_[index] == lethal_val_)
    {
      wall_index_ = index;
      hit_wall_ = true;
    }

  }

  inline unsigned int getSizeX() { return size_x_; }

  inline unsigned int getWallIndex() { return wall_index_; }

  inline unsigned int didHitWall() { return hit_wall_; }

private:

  unsigned char * costmap_;

  unsigned char lethal_val_;

  unsigned int size_x_;

  unsigned int wall_index_;

  bool hit_wall_;

};

class WallFollowingCostFunction : public base_local_planner::TrajectoryCostFunction
{

public:

  WallFollowingCostFunction(costmap_2d::Costmap2D * costmap);

  virtual ~WallFollowingCostFunction();

  bool prepare();
  
  double scoreTrajectory(base_local_planner::Trajectory & traj);

  inline void setTargetDistanceToWall(double target_dist_to_wall) { target_dist_to_wall_ = target_dist_to_wall; }

  inline void setMaxRayTraceDistance(double max_raytrace_dist) { max_raytrace_dist_ = max_raytrace_dist; }

private:

  double calculateDistanceToWall(double traj_x, double traj_y, double traj_th);

  double costmapIndexToDistance(unsigned int costmap_index, double traj_x, double traj_y);

  double scoreDistanceToWall(double dist);

  void raytraceLine(WallFollowingRayTracer & rt, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int max_length = UINT_MAX);

  void bresenham2D(WallFollowingRayTracer & rt, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b, unsigned int offset, unsigned int max_length);

  inline int sign(int x)
  {
    return x > 0 ? 1.0 : -1.0;
  }


  costmap_2d::Costmap2D * costmap_;

  WallFollowingSide wall_following_side_;

  double target_dist_to_wall_;

  double max_raytrace_dist_;

};

} // namespace wall_following

#endif // #ifndef WALL_FOLLOWING_COST_FUNCTION_H_


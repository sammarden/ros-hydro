
#include <gtest/gtest.h>

#include <local_planner/local_planner_ros.h>

TEST(ConstantLinearVelocityTrajectoryGenerator, generateTrajectories)
{

  local_planner::ConstantLinearVelocityTrajectoryGenerator traj_gen;

  traj_gen.setLinearVel(1.0);
  traj_gen.setAngularVelMin(-1.0);
  traj_gen.setAngularVelMax(1.0);
  traj_gen.setSimTime(1.0);
  traj_gen.setSimDt(0.1);

  std::vector<std::pair<geometry_msgs::Twist, base_local_planner::Trajectory> > cmd_vel_trajs;

  traj_gen.setAngularVelSamples(3);
  traj_gen.setAngularVelResamples(0);
  traj_gen.generateTrajectories(cmd_vel_trajs);
  EXPECT_EQ(cmd_vel_trajs.size(), 3);
  EXPECT_EQ(cmd_vel_trajs[0].second.getPointsSize(), 10);

  cmd_vel_trajs.clear();

  traj_gen.setAngularVelSamples(1);
  traj_gen.setAngularVelResamples(3);
  traj_gen.generateTrajectories(cmd_vel_trajs);
  EXPECT_EQ(cmd_vel_trajs.size(), 1);

  cmd_vel_trajs.clear();

  traj_gen.setAngularVelSamples(3);
  traj_gen.setAngularVelResamples(1);
  traj_gen.generateTrajectories(cmd_vel_trajs);
  EXPECT_EQ(cmd_vel_trajs.size(), 9);
  for (int i = 0; i < cmd_vel_trajs.size(); ++i)
  {
    EXPECT_EQ(cmd_vel_trajs[i].second.getPointsSize(), 10);
  }

}

TEST(CmdVelSampler, reset_next_isEmpty)
{

  local_planner::CmdVelSampler cmd_vel_sampler;

  cmd_vel_sampler.reset(0.0, 10.0, 3, -10.0, 10.0, 3);

  geometry_msgs::Twist cmd_vel;

  cmd_vel = cmd_vel_sampler.next();
  EXPECT_FLOAT_EQ(cmd_vel.linear.x, 0.0);
  EXPECT_FLOAT_EQ(cmd_vel.angular.z, -10.0);

  cmd_vel = cmd_vel_sampler.next();
  EXPECT_FLOAT_EQ(cmd_vel.linear.x, 0.0);
  EXPECT_FLOAT_EQ(cmd_vel.angular.z, 0.0);

  cmd_vel = cmd_vel_sampler.next();
  EXPECT_FLOAT_EQ(cmd_vel.linear.x, 0.0);
  EXPECT_FLOAT_EQ(cmd_vel.angular.z, 10.0);

  cmd_vel = cmd_vel_sampler.next();
  EXPECT_FLOAT_EQ(cmd_vel.linear.x, 5.0);
  EXPECT_FLOAT_EQ(cmd_vel.angular.z, -10.0);

  cmd_vel = cmd_vel_sampler.next();
  EXPECT_FLOAT_EQ(cmd_vel.linear.x, 5.0);
  EXPECT_FLOAT_EQ(cmd_vel.angular.z, 0.0);

  cmd_vel = cmd_vel_sampler.next();
  EXPECT_FLOAT_EQ(cmd_vel.linear.x, 5.0);
  EXPECT_FLOAT_EQ(cmd_vel.angular.z, 10.0);

  cmd_vel = cmd_vel_sampler.next();
  EXPECT_FLOAT_EQ(cmd_vel.linear.x, 10.0);
  EXPECT_FLOAT_EQ(cmd_vel.angular.z, -10.0);

  cmd_vel = cmd_vel_sampler.next();
  EXPECT_FLOAT_EQ(cmd_vel.linear.x, 10.0);
  EXPECT_FLOAT_EQ(cmd_vel.angular.z, 0.0);

  cmd_vel = cmd_vel_sampler.next();
  EXPECT_FLOAT_EQ(cmd_vel.linear.x, 10.0);
  EXPECT_FLOAT_EQ(cmd_vel.angular.z, 10.0);

  EXPECT_TRUE(cmd_vel_sampler.isEmpty());

  cmd_vel_sampler.reset(0.0, 0.0, 1, 0.0, 1.0, 3);

  cmd_vel = cmd_vel_sampler.next();
  cmd_vel = cmd_vel_sampler.next();
  cmd_vel = cmd_vel_sampler.next();
  EXPECT_TRUE(cmd_vel_sampler.isEmpty());

  cmd_vel_sampler.reset(0.0, 1.0, 3, 0.0, 0.0, 1);

  cmd_vel = cmd_vel_sampler.next();
  cmd_vel = cmd_vel_sampler.next();
  cmd_vel = cmd_vel_sampler.next();
  EXPECT_TRUE(cmd_vel_sampler.isEmpty());

  cmd_vel_sampler.reset(0.0, 0.0, 1, 0.0, 0.0, 1);

  cmd_vel = cmd_vel_sampler.next();
  EXPECT_TRUE(cmd_vel_sampler.isEmpty());



}

int main(int argc, char ** argv)
{

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();

}

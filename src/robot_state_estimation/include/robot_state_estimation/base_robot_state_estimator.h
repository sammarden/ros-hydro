
#ifndef BASE_ROBOT_STATE_ESTIMATOR_H_
#define BASE_ROBOT_STATE_ESTIMATOR_H_

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <boost/thread/thread.hpp>

namespace robot_state_estimation
{

/** \brief Implements a base class for robot state estimation. Ensures that the estimator publishes its estimation at a given rate.
  *
  * Authors: Sam Marden
  */
class BaseRobotStateEstimator
{

public:

  /** \brief Default constructor. */
  BaseRobotStateEstimator();

  /** \brief Destructor. Shuts down loop thread if it hasn't already been shut down. */
  virtual ~BaseRobotStateEstimator();

protected:

  /** \brief Start up the loop thread. */
  void startTFLoop();

  /** \brief Shut down the loop thread. */
  void stopTFLoop();

  /** \brief Set the state to be broadcast.
    *
    * x X estimate to be broadcast.
    * y Y estimate to be broadcast.
    * z Z estimate to be broadcast.
    * roll Roll estimate to be broadcast.
    * pitch Pitch estimate to be broadcast.
    * yaw Yaw estimate to be broadcast.
    */
  void setTFState(double x, double y, double z, double roll, double pitch, double yaw);

  /** \brief The map frame. */
  std::string map_frame_;

  /** \brief The base frame of the robot. */
  std::string robot_base_frame_;

  /** \brief The frequency at which the loop should be running. */
  double tf_rate_;

private:

  /** \brief The publishing loop. Calls the loop callback funciton. */
  void tfLoop();

  /** \brief Broadcast the TF transform from the state estimate of the robot. */
  void broadcastTransform();

  /** \brief Transform broadcaster for the map -> base_footprint transform. */
  tf::TransformBroadcaster tf_broadcaster_;

  /** \brief Publishing loop thread. */
  boost::shared_ptr<boost::thread> tf_loop_thread_ptr_;

  /** \brief Mutex for the state to be broadcast. */
  boost::mutex tf_state_lock_;

  /** \brief X estimate to be broadcast. */
  double tf_x_;

  /** \brief Y estimate to be broadcast. */
  double tf_y_;

  /** \brief Z estimate to be broadcast. */
  double tf_z_;

  /** \brief Roll estimate to be broadcast. */
  double tf_roll_;

  /** \brief Pitch estimate to be broadcast. */
  double tf_pitch_;

  /** \brief Yaw estimate to be broadcast. */
  double tf_yaw_;

  /** \brief Whether the loop should still be running. */
  bool tf_looping_;

};

} // namespace robot_state_estimation

#endif // #ifndef BASE_ROBOT_STATE_ESTIMATOR_H_



#pragma once

#include <cmath>
#include <mutex>
#include <thread>
// My libraries
#include "pick_and_place_libfranka/TrajectoryHelper.h"

// Third party libraries
#include <TooN/TooN.h>

// Librerie Libfranka
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

// Contiene definizioni di funzioni utili, presente in libfranka
#include <pick_and_place_libfranka/examples_common.h>

// Action and srv
#include <actionlib/server/simple_action_server.h>
#include <pick_and_place_libfranka/CartesianTrajectoryAction.h>
#include <pick_and_place_libfranka/JointPointTrajectoryAction.h>
#include <pick_and_place_libfranka/JointTrajectoryAction.h>

// msgs
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class TrajAction {

protected:
  ros::NodeHandle nh_;
  std::shared_ptr<franka::Robot> robot_;
  std::mutex *robot_mutex_;
  ros::Publisher state_pub_;
  std::unique_ptr<std::thread> publish_thread_;
  bool publish_command = false;
  bool debugging = false;

  actionlib::SimpleActionServer<
      pick_and_place_libfranka::JointPointTrajectoryAction>
      joint_point_traj_as_;

  actionlib::SimpleActionServer<pick_and_place_libfranka::JointTrajectoryAction>
      joint_traj_as_;

  actionlib::SimpleActionServer<
      pick_and_place_libfranka::CartesianTrajectoryAction>
      cartesian_traj_as_;

  void publish_state();

public:
  TrajAction(std::shared_ptr<franka::Robot> panda, std::mutex *robot_mutex);

  ~TrajAction(void);

  void JointPointTrajCB(
      const pick_and_place_libfranka::JointPointTrajectoryGoalConstPtr &goal);

  void JointTrajCB(
      const pick_and_place_libfranka::JointTrajectoryGoalConstPtr &goal);

  void CartesianTrajCB(
      const pick_and_place_libfranka::CartesianTrajectoryGoalConstPtr &goal);
};

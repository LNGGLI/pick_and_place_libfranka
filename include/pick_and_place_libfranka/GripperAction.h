
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
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

// Contiene definizioni di funzioni utili, presente in libfranka
#include <pick_and_place_libfranka/examples_common.h>

// Action and srv
#include <actionlib/server/simple_action_server.h>
#include <pick_and_place_libfranka/GripperMoveAction.h>

// msgs
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class GripperAction {

protected:
  ros::NodeHandle nh_;
  std::shared_ptr<franka::Robot> robot_;
  std::unique_ptr<franka::Gripper> gripper_;
  std::shared_ptr<std::mutex> robot_mutex_;

  bool publish_command = false;
  bool debugging = false;

  actionlib::SimpleActionServer<pick_and_place_libfranka::GripperMoveAction>
      gripper_move_as_;

public:
  GripperAction(std::shared_ptr<franka::Robot> panda,
                std::shared_ptr<std::mutex> robot_mutex);

  ~GripperAction(void);

  void
  GripperMoveCB(const pick_and_place_libfranka::GripperMoveGoalConstPtr &goal);
};

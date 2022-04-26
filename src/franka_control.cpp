// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// Librerie standard
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <thread>
#include <vector>

// Custom functions

#include "pick_and_place_libfranka/franka_control.h"
#include <pick_and_place_libfranka/check_realtime.h>

// Libreria Eigen Dense
#include <Eigen/Dense>

// Librerie Libfranka
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

// Contiene definizioni di funzioni utili, presente in libfranka
#include <pick_and_place_libfranka/examples_common.h>

// Utils
#include <TooN/TooN.h>
#include <pick_and_place_libfranka/check_realtime.h>
#include <ros/ros.h>

// Messaggi, servizi e azioni
#include <actionlib/server/simple_action_server.h>
#include <pick_and_place_libfranka/GripperAction.h>
#include <pick_and_place_libfranka/JointPointTrajectoryAction.h>
#include <pick_and_place_libfranka/TrajAction.h>
#include <sensor_msgs/JointState.h>

/**

  Documentazione classe franka::Robot
  https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html
  Documentazione classe franka::Model:
  https://frankaemika.github.io/libfranka/classfranka_1_1Model.html
  Documentazione classe franka::RobotState:
  https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html
**/

using namespace franka_node;

int main(int argc, char **argv) {

  ros::init(argc, argv, "franka_node");
  ros::NodeHandle nh;
  std::string robot_IP;

  // Robot object creation
  if (!nh.getParam("robot_ip", robot_IP)) {
    ROS_ERROR_STREAM("Param robot_ip not found.");
  }

  std::shared_ptr<franka::Robot> panda =
      std::make_shared<franka::Robot>(robot_IP);

  std::mutex robot_mutex;
  // Start Action Servers for control
  TrajAction traj_action(panda, &robot_mutex);

  // Start Action Servers for gripper
  // GripperAction gripper_action(panda, robot_mutex);

  ros::spin();

  return 0;
}

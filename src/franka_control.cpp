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

  Nota3: la documentazione della libreria Eigen3 è reperibile al link
  https://eigen.tuxfamily.org/dox/. Se si è familiari con l'utilizzo del Matlab
  si consiglia di dare uno sguardo al seguente cheat sheet:
  https://eigen.tuxfamily.org/dox/AsciiQuickReference.txt

 */

using namespace franka_node;
using JointPointActionServer = actionlib::SimpleActionServer<
    pick_and_place_libfranka::JointPointTrajectoryAction>;

int main(int argc, char **argv) {

  ros::init(argc, argv, "franka_node");
  ros::NodeHandle nh;
  TrajAction traj_action;

  ros::spin();

  return 0;
}

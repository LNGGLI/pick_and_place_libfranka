// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// Librerie standard
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>

// Custom functions

#include "pick_and_place_libfranka/franka_node.h"

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

// Messaggi e servizi

#include <pick_and_place_libfranka/SetState.h>
#include <pick_and_place_libfranka/TrajectoryPointStamped.h>

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

using namespace franka_control;

int main(int argc, char **argv) {

  ros::init(argc, argv, "libfranka_controller_node");
  ros::NodeHandle nh;

  std::string robot_IP;
  if (!nh.getParam("robot_ip", robot_IP)) {
    ROS_ERROR_STREAM("Specificare l'indirizzo IP del robot.");
    return -1;
  }

  ros::Subscriber command_sub =
      nh.subscribe<pick_and_place_libfranka::TrajectoryPointStamped>(
          "/joint_commands", 1, &franka_control::CommandCB);
  ros::ServiceClient client_setstate =
      nh.serviceClient<pick_and_place_libfranka::SetState>("set_state");

  client_setstate
      .waitForExistence(); // Attende che venga istanziato il servizio

  try {

    // Connessione al robot
    franka::Robot robot(robot_IP);

    // Set collision behavior
    // (https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html#a168e1214ac36d74ac64f894332b84534)
    setDefaultBehavior(robot);

    // Modello cinematico e dinamico del robot
    franka::Model model = robot.loadModel();

    // Lettura dello stato attuale del robot e invio tramite srv al trajectory
    // planner
    franka::RobotState initial_state = robot.readOnce();
    pick_and_place_libfranka::SetState setstate_msg;

    for (int i = 0; i < 7; i++) {
      setstate_msg.request.initial_configuration[i] = initial_state.q[i];
      q_initial_[i] = initial_state.q[i]; // inizializzo q_command con la configurazione iniziale.
    }

    if (client_setstate.call(setstate_msg)) {
      ROS_INFO("Success: %d. Messagge: %s", setstate_msg.response.success,
               setstate_msg.response.message.c_str());
    } else {
      ROS_INFO("Failed to call service SetState");
      return -1;
    }

    // Doc:
    // https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html#a5b5ba0a4f2bfd20be963b05622e629e1

    
    ros::Rate loop_rate(1000.0);

    robot.control([&loop_rate] (const franka::RobotState &, franka::Duration period ) -> franka::JointPositions {
      
      
      ros::spinOnce(); // lettura dei comandi attraverso CommandCB

      if (q_command_[0] != 0.0) {

        std::cout << "Ricevuta q_command_ non vuota \n";
        if (ros::ok() && !motion_finished_)
          return franka::JointPositions(q_command_);
        else {
          std::cout << "Ricevuto motion finished << " << motion_finished_
                    << "\n";
          return franka::MotionFinished(franka::JointPositions(q_command_));
        }

      } 
      else {

        // std::cout << "Ricevuta q_command_ vuota \n";
        if (ros::ok() && !motion_finished_)
          return franka::JointPositions(q_initial_);
        else {
          std::cout << "Ricevuto motion finished << " << motion_finished_
                    << "\n";
          return franka::MotionFinished(franka::JointPositions(q_initial_));
        }

      }

      loop_rate.sleep(); // forzo la frequenza del loop di controllo

    });

  }

  catch (const franka::Exception &ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}

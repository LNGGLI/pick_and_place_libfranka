
#pragma once

// Libfranka
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

// utils
#include <ros/ros.h>

// Messaggi

#include <pick_and_place_libfranka/TrajectoryPointStamped.h>

namespace franka_control {

std::array<double, 7> q_command_;
std::array<double, 7> q_initial_;
std::array<double, 7> qp_command_;
bool motion_finished_ = false;


// franka::JointPositions
// joint_position_callback(const franka::RobotState &robot_state,
//                         franka::Duration period) {

//   ros::spinOnce(); // lettura dei comandi attraverso CommandCB
//   std::cout << "Period = " << period.toSec() << "\n";
//   // std::cout << "Q_comandata = \n";
//   // for (int i = 0; i < 7; i++) {
//   //   std::cout << q_command_[i] << " ";
//   // }
//   // std::cout << "\n";

//   if (ros::ok() && !motion_finished_)
//     return franka::JointPositions(q_initial_);
//   else
//     return franka::MotionFinished(franka::JointPositions(q_initial_));

//   // franka::JointPositions q_des(q_command_);

//   // if (motion_finished_ || !ros::ok()) {
//   //   std::cout << "Motion finished comandato \n";
//   //   return franka::MotionFinished(q_des);
//   // } else {
//   //   return q_des;
//   // }
//   loop_rate.sleep();
// }

void CommandCB(pick_and_place_libfranka::TrajectoryPointStampedConstPtr msg) {

  for (int i = 0; i < 7; i++) {
    q_command_[i] = msg->point.positions[i];

    // qp_command_[i] = msg->point.velocities[i];
  }

  motion_finished_ = msg->finished;

  // std::cout << "Ho ricevuto configurazione Q = ";
  // for (int i = 0; i < 7; i++)
  //   std::cout << q_command_[i] << " ";

  // std::cout << "\n Con motion_finished = " << motion_finished_ << "\n";
}

} // namespace franka_control
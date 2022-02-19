
#pragma once

// Libfranka
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>



// Messaggi

#include <pick_and_place_libfranka/TrajectoryPointStamped.h>

namespace franka_control{

std::array<double,7> q_command_;



franka::JointPositions joint_position_callback(const franka::RobotState &, franka::Duration){

  //if (time >= 2 * time_max) {

    // std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
    // return franka::MotionFinished(velocities);

  //}
  franka::JointPositions q_d(q_command_);
  return q_d;

}

void CommandCB(pick_and_place_libfranka::TrajectoryPointStampedConstPtr msg){

  for(int i = 0 ; i < 7; i++){
    q_command_[i] = msg->point.positions[i];
  }
  

}




}
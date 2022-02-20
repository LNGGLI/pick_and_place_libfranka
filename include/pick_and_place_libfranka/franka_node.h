
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

namespace franka_control{

std::array<double,7> q_command_;
std::array<double,7> qp_command_;
bool motion_finished = false;



franka::JointPositions joint_position_callback(const franka::RobotState &, franka::Duration){

  ros::spinOnce(); // lettura dei comandi attraverso CommandCB
  franka::JointPositions q_des(q_command_);

  if(motion_finished){
    return franka::MotionFinished(q_des);
  }
  else{
    return q_des;
  }

}

void CommandCB(pick_and_place_libfranka::TrajectoryPointStampedConstPtr msg){

  for(int i = 0 ; i < 7; i++){
    q_command_[i] = msg->point.positions[i];
    qp_command_[i] = msg->point.velocities[i];
  }

  motion_finished = msg->finished;


  

}




}
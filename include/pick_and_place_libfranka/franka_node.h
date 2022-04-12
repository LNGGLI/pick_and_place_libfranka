
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
#include <sensor_msgs/JointState.h>

namespace franka_node {

void publish_state(ros::NodeHandle nh) {

  std::string robot_IP;
  if (!nh.getParam("robot_ip", robot_IP)) {
    ROS_ERROR_STREAM("Specificare l'indirizzo IP del robot.");
    // as_.setAborted();
  }

 

  try {

     franka::Robot robot(robot_IP);

  ros::Publisher command_pub =
      nh.advertise<sensor_msgs::JointState>("/joint_state", 1);

  sensor_msgs::JointState msg;
  msg.name = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};
  std::array<double, 7UL> joint_state{};

  while (ros::ok()) {

    msg.header.stamp = ros::Time::now();

    msg.position.clear();
    joint_state = robot.readOnce().q;
    for (int i = 0; i < 7; i++)
      msg.position.push_back(joint_state[i]);

    command_pub.publish(msg);
  }

 
  }

  catch (const franka::Exception &ex) {
    // print exception
    std::cout << ex.what()  << "inside thread" << std::endl;
  }
}

} // namespace franka_node
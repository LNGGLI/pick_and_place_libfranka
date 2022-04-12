#pragma once


// Librerie Libfranka
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

// Contiene definizioni di funzioni utili, presente in libfranka
#include <pick_and_place_libfranka/examples_common.h>

// msgs, action and srv
#include <actionlib/server/simple_action_server.h>
#include <pick_and_place_libfranka/JointPointTrajectoryAction.h>
#include <pick_and_place_libfranka/JointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>



class JointPointTrajAction {

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<pick_and_place_libfranka::JointPointTrajectoryAction> as_;
  pick_and_place_libfranka::JointPointTrajectoryFeedback feedback_;
  pick_and_place_libfranka::JointPointTrajectoryResult result_;
  std::string action_name_;

public: 

  JointPointTrajAction(std::string name);

  ~JointPointTrajAction(void);

  void executeCB(const pick_and_place_libfranka::JointPointTrajectoryGoalConstPtr &goal);

}; // class JointPointTrajAction


class JointTrajAction {

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<pick_and_place_libfranka::JointTrajectoryAction> as_;
  pick_and_place_libfranka::JointTrajectoryFeedback feedback_;
  pick_and_place_libfranka::JointTrajectoryResult result_;
  std::string action_name_;

public: 

  JointTrajAction(std::string name);

  ~JointTrajAction(void);

  void executeCB(const pick_and_place_libfranka::JointTrajectoryGoalConstPtr &goal);

}; // class JointTrajAction

#pragma once

#include <mutex>
#include <thread>


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
#include <sensor_msgs/JointState.h>




class TrajAction {

protected:

  ros::NodeHandle nh_;
  std::shared_ptr<franka::Robot> robot_;
  std::mutex robot_mutex_;
  ros::Publisher state_pub_;
  std::unique_ptr<std::thread> publish_thread_;

  actionlib::SimpleActionServer<
      pick_and_place_libfranka::JointPointTrajectoryAction>
      joint_point_traj_as_;
  pick_and_place_libfranka::JointPointTrajectoryFeedback feedback_;
  pick_and_place_libfranka::JointPointTrajectoryResult result_;

  void publish_state();

public:

  TrajAction();

  ~TrajAction(void);

  void JointPointTrajCB(
      const pick_and_place_libfranka::JointPointTrajectoryGoalConstPtr &goal);
};





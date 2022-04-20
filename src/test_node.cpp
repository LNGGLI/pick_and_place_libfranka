
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <pick_and_place_libfranka/Panda.h>
#include <pick_and_place_libfranka/trajectory_node.h>

// Server , Actions
#include <actionlib/client/simple_action_client.h>
#include <pick_and_place_libfranka/JointTrajectoryAction.h>

// Utils
#include <TooN/TooN.h>
#include <pick_and_place_libfranka/check_realtime.h>
#include <ros/ros.h>

// Messages e Srv

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "trajectory_planner");
  ros::NodeHandle nh;

  // if (!check_realtime())
  //   throw std::runtime_error("REALTIME NOT AVAILABLE");

  // if (!set_realtime_SCHED_FIFO())
  //   throw std::runtime_error("ERROR IN set_realtime_SCHED_FIFO");

  actionlib::SimpleActionClient<pick_and_place_libfranka::JointTrajectoryAction>
      trajectory_ac("joint_traj", true);

  pick_and_place_libfranka::JointTrajectoryGoal goal;
  trajectory_msgs::JointTrajectoryPoint point;
  std::array<double, 7> q_goal{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  std::array<double, 7> q_init{{0, 0, 0, 0, 0, 0, 0}};

  // ros::Rate loop_rate(10);
  // while (ros::ok() && !initial_state_acquired) {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  double t = 0.0;
  double tf = 10.0;
  ros::Rate loop_rate(1000);
  std::array<double, 7> q_command{};
  while (ros::ok() && t < tf) {

    t = t + 0.001;
    double tau = t / tf;
    for (int i = 0; i < 7; i++)
      q_command[i] = q_init[i] + (q_goal[i] - q_init[i]) *
                                     (6 * pow(tau, 5) - 15 * pow(tau, 4) +
                                      10 * pow(tau, 3));

    for (int i = 0; i < 7; i++)
      point.positions.push_back(q_command[i]);
    point.time_from_start = ros::Duration(t);

    goal.trajectory.points.push_back(point);
    point.positions.clear();
  }

  trajectory_ac.waitForServer();

  trajectory_ac.sendGoal(goal);

  bool finished_before_timeout =
      trajectory_ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout) {
    pick_and_place_libfranka::JointTrajectoryResultConstPtr result =
        trajectory_ac.getResult();
    ROS_INFO("Action finished: %d", result->success);
  } else
    ROS_INFO("Action did not finish before the time out.");

  // exit
  return 0;
}

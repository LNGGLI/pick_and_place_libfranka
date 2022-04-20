
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <pick_and_place_libfranka/Panda.h>
#include <pick_and_place_libfranka/trajectory_node.h>

// Server , Actions
#include <actionlib/client/simple_action_client.h>
#include <pick_and_place_libfranka/CartesianTrajectoryAction.h>
#include <pick_and_place_libfranka/JointTrajectoryAction.h>

// Utils
#include <TooN/TooN.h>
#include <pick_and_place_libfranka/check_realtime.h>
#include <ros/ros.h>

// Sun
#include "sun_math_toolbox/PortingFunctions.h"
#include <sun_traj_lib/Cartesian_Independent_Traj.h>
#include <sun_traj_lib/Line_Segment_Traj.h>
#include <sun_traj_lib/Quintic_Poly_Traj.h>
#include <sun_traj_lib/Rotation_Const_Axis_Traj.h>

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

  /********** Test JointTrajectoryAction*************/
  // actionlib::SimpleActionClient<pick_and_place_libfranka::JointTrajectoryAction>
  //     trajectory_ac("joint_traj", true);

  // pick_and_place_libfranka::JointTrajectoryGoal goal;
  // trajectory_msgs::JointTrajectoryPoint point;
  // std::array<double, 7> q_goal{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2,
  // M_PI_4}}; std::array<double, 7> q_init{{0, 0, 0, 0, 0, 0, 0}};

  double t = 0.0;
  // double tf = 10.0;
  // ros::Rate loop_rate(1000);
  // std::array<double, 7> q_command{};

  // while (ros::ok() && t < tf) {

  //   t = t + 0.001;
  //   double tau = t / tf;
  //   for (int i = 0; i < 7; i++)
  //     q_command[i] = q_init[i] + (q_goal[i] - q_init[i]) *
  //                                    (6 * pow(tau, 5) - 15 * pow(tau, 4) +
  //                                     10 * pow(tau, 3));

  //   for (int i = 0; i < 7; i++)
  //     point.positions.push_back(q_command[i]);
  //   point.time_from_start = ros::Duration(t);

  //   goal.trajectory.points.push_back(point);
  //   point.positions.clear();
  // }

  // trajectory_ac.waitForServer();

  // trajectory_ac.sendGoal(goal);

  // bool finished_before_timeout =
  //     trajectory_ac.waitForResult(ros::Duration(30.0));

  // if (finished_before_timeout) {
  //   pick_and_place_libfranka::JointTrajectoryResultConstPtr result =
  //       trajectory_ac.getResult();
  //   ROS_INFO("Action finished: %d", result->success);
  // } else
  //   ROS_INFO("Action did not finish before the time out.");

  /********** Test CartesianTrajectoryAction*************/
  std::cout << "Invocazione dell'action cartesian_traj\n";

  actionlib::SimpleActionClient<
      pick_and_place_libfranka::CartesianTrajectoryAction>
      cartesian_ac("cartesian_traj", true);

  ros::Publisher cartesian_traj_publisher =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
          "cartesian_traj_desired", 1);

  // create cartesian traj
  // Create robot object
  TooN::Matrix<4, 4, double> n_T_e = TooN::Identity(4);
  sun::Panda panda(n_T_e, 1.0, "panda");

  pick_and_place_libfranka::CartesianTrajectoryGoal cartesian_goal;
  TooN::Vector<7, double> starting_q =
      TooN::makeVector(0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4);
  TooN::Matrix<4, 4, double> starting_pose = panda.fkine(starting_q);
  TooN::Vector<3, double> starting_position = sun::transl(starting_pose);
  std::cout << "Starting position: " << starting_position << "\n";
  sun::UnitQuaternion starting_quat(starting_pose);

  std::cout << "Creazione cartesian traj\n";
  cartesian_goal.trajectory.points.reserve(10000);

  t = 0.0;
  ros::Rate lp(1000);
  for (int i = 0; i < 1000; i++) {

    geometry_msgs::Transform transf;
    transf.translation.x = starting_position[0] + 0.0001 * i;
    transf.translation.y = starting_position[1] + 0.0001 * i;
    transf.translation.z = starting_position[2];
    transf.rotation.w = starting_quat.getS();
    transf.rotation.x = starting_quat.getV()[0];
    transf.rotation.y = starting_quat.getV()[1];
    transf.rotation.z = starting_quat.getV()[2];

    trajectory_msgs::MultiDOFJointTrajectoryPoint point;
    point.transforms.push_back(transf);
    point.time_from_start = ros::Duration(t);

    cartesian_goal.trajectory.points.push_back(point);

    cartesian_traj_publisher.publish(point);

    t = t + 0.001;

    lp.sleep();
  }

  std::cout << "Traiettoria completata. Invocazione del servizio in corso\n";
  cartesian_ac.waitForServer();
  cartesian_ac.sendGoal(cartesian_goal);

  bool finished_before_timeout =
      cartesian_ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout) {
    pick_and_place_libfranka::CartesianTrajectoryResultConstPtr result =
        cartesian_ac.getResult();
    ROS_INFO("Action finished: %d", result->success);
  } else
    ROS_INFO("Action did not finish before the time out.");

  return 0;
}

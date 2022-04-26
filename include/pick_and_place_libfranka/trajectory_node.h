
#pragma once

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <pick_and_place_libfranka/Panda.h>

// Server , Actions
#include <actionlib/client/simple_action_client.h>
#include <pick_and_place_libfranka/CartesianTrajectoryAction.h>
#include <pick_and_place_libfranka/GripperAction.h>
#include <pick_and_place_libfranka/JointTrajectoryAction.h>
// Utils
#include <TooN/TooN.h>
#include <pick_and_place_libfranka/check_realtime.h>
#include <ros/ros.h>

// Sun
#include <sun_robot_lib/Robot.h>
#include <sun_traj_lib/Cartesian_Independent_Traj.h>
#include <sun_traj_lib/Line_Segment_Traj.h>
#include <sun_traj_lib/Quintic_Poly_Traj.h>
#include <sun_traj_lib/Rotation_Const_Axis_Traj.h>

// Messages e Srv
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

bool state_read = false;
std::array<double, 7> current_state;

// Struct CartesianPoint
struct CartesianPoint {
  TooN::Vector<3, double> position;
  sun::UnitQuaternion quaternion;
  double Tf;
};

void stateCallBack(const sensor_msgs::JointState::ConstPtr &msg) {
  for (int i = 0; i < 7; i++)
    current_state[i] = msg->position[i];
  state_read = true;
}

/**
 * @brief Computes the cartesian trajectory to reach the desired point in the
 * specified time. The cartesian trajectory is defined as a line segment and a
 * rotation about a constant axis.
 *
 * @param point Desired point expressed as a trajectory::CartresianPoint.
 * @param panda The sun::Panda object that represents the panda robot.
 * @return The full cartesian trajectory ready to be sent as a goal.
 */
pick_and_place_libfranka::CartesianTrajectoryGoal
CartesianPoint2CartesianTrajGoal(const CartesianPoint &point,
                                 const sun::Panda &panda) {

  pick_and_place_libfranka::CartesianTrajectoryGoal cartesian_traj_goal;
  // Define temporal distance between points.

  // Desired Tf
  double Tf = point.Tf;
  // Desired Pose
  TooN::Vector<3, double> pf = point.position;
  sun::UnitQuaternion final_quat(point.quaternion); // desired quaternion

  // Compute Ts
  double Ts = 1;
  int N_points = Tf / Ts;

  // Get initial configuration
  ros::Rate slow_loop(100);
  while (ros::ok() && !state_read) {
    ros::spinOnce();
    slow_loop.sleep();
  }

  // Initial pose
  TooN::Vector<7, double> current_joint_state;
  for (int i = 0; i < 7; i++)
    current_joint_state[i] = current_state[i];
  TooN::Matrix<4, 4, double> initial_pose = panda.fkine(current_joint_state);
  TooN::Vector<3> pi = sun::transl(initial_pose); // initial position
  sun::UnitQuaternion init_quat(initial_pose);    // initial orientation

  // Nota: la traiettoria in orientamento è quella definita dal Delta_quat.
  // Perchè vogliamo andare da init_quat (orientamento iniziale) a final_quat
  // (orientamento finale). Il metodo getquaternion(t) restituisce DeltaQuat(t)
  // * init_quat. Il metodo getPosition(t) resituisce direttamente la posizione
  // corretta tra pi e pf.
  sun::UnitQuaternion delta_quat = final_quat * inv(init_quat);
  sun::AngVec angvec = delta_quat.toangvec();

  // Build line segment traj + Rotation Const Axis

  sun::Quintic_Poly_Traj qp_position(Tf, 0.0, 1.0); // quintic poly
  sun::Quintic_Poly_Traj qp_orientation(Tf, 0.0, angvec.getAng());

  sun::Line_Segment_Traj line_traj(pi, pf, qp_position);
  sun::Rotation_Const_Axis_Traj quat_traj(init_quat, angvec.getVec(),
                                          qp_orientation);

  sun::Cartesian_Independent_Traj cartesian_traj(line_traj, quat_traj);

  cartesian_traj_goal.trajectory.points.reserve(N_points + 1);

  trajectory_msgs::MultiDOFJointTrajectoryPoint cartesian_point;

  TooN::Vector<3, double> posizione_d;
  sun::UnitQuaternion unit_quat_d;
  double t = 0.0;
  while (ros::ok() && !cartesian_traj.isCompleate(t)) {

    t = t + Ts; // tempo trascorso
    posizione_d = cartesian_traj.getPosition(t);
    unit_quat_d = cartesian_traj.getQuaternion(t);

    geometry_msgs::Transform transf;
    transf.translation.x = posizione_d[0];
    transf.translation.y = posizione_d[1];
    transf.translation.z = posizione_d[2];
    transf.rotation.w = unit_quat_d.getS();
    transf.rotation.x = unit_quat_d.getV()[0];
    transf.rotation.y = unit_quat_d.getV()[1];
    transf.rotation.z = unit_quat_d.getV()[2];

    trajectory_msgs::MultiDOFJointTrajectoryPoint cartesian_point;
    cartesian_point.transforms.push_back(transf);
    cartesian_point.time_from_start = ros::Duration(t);

    cartesian_traj_goal.trajectory.points.push_back(cartesian_point);
  }
  return cartesian_traj_goal;
}

bool set_goal_and_call_action(const CartesianPoint &cartesian_goal) {

  // Client creation
  static actionlib::SimpleActionClient<
      pick_and_place_libfranka::CartesianTrajectoryAction>
      cartesian_traj_ac("cartesian_traj", true);
  cartesian_traj_ac.waitForServer();

  // Panda object creation ( needed for computing the trajectory)
  // TooN::Matrix<4, 4, double> n_T_e(TooN::Data(0.7071, 0.7071, 0.0, 0.0,
  //                                             -0.7071,
  //                                             0.7071, 0.0, 0.0, 0.0,
  //                                             0.0, 1.0, 0.1034, 0.0, 0.0,
  //                                             0.0, 1.0));
  // Quando viene montato il gripper assicurarsi che pz sia impostato su
  // AppDesk a 0.1034 prima di utilizzare questa matrice di trasformazione

  static TooN::Matrix<4, 4, double> n_T_e = TooN::Identity(4);
  static sun::Panda panda(n_T_e, 1.0, "panda");

  pick_and_place_libfranka::CartesianTrajectoryGoal cartesian_traj_goal;
  cartesian_traj_goal = CartesianPoint2CartesianTrajGoal(cartesian_goal, panda);
  std::cout << "Cartesian Traj Goal created. Sending goal \n";
  cartesian_traj_ac.sendGoal(cartesian_traj_goal);

  std::cout << "Waiting for result \n";
  bool finished_before_timeout =
      cartesian_traj_ac.waitForResult(ros::Duration(cartesian_goal.Tf * 3));

  pick_and_place_libfranka::CartesianTrajectoryResultConstPtr result;
  if (finished_before_timeout) {
    result = cartesian_traj_ac.getResult();
    ROS_INFO("Action finished, success :  %d", result->success);
  } else
    ROS_INFO("Action did not finish before the time out.");
  return result->success;
}

/**
 * @brief Commands the gripper to move to the desired width at the desired
 * velocity.
 *
 * @param width desired width between fingers
 * @param speed desired moving speed
 * @return true if the gripper moved.
 * @return false if the gripper did not move.
 */
bool gripper_move(const double &width, const double &speed) {

  actionlib::SimpleActionClient<pick_and_place_libfranka::GripperMoveAction>
      move_client("gripper_move");
  move_client.waitForServer();
  pick_and_place_libfranka::GripperMoveGoal move_goal;
  move_goal.desired_speed = speed;
  move_goal.desired_width = width;

  move_client.sendGoal(move_goal);
  bool finished_before_timeout = move_client.waitForResult(ros::Duration(10.0));

  if (finished_before_timeout) {
    pick_and_place_libfranka::GripperMoveResultConstPtr result =
        move_client.getResult();
    if (result->success != true) {
      std::cout << "Move failed \n";
    }

    return result->success;
  } else {
    std::cout << "Move action did not finish before the time out. \n";
    return false;
  }
}

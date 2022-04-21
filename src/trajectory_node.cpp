
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

using namespace trajectory;

int main(int argc, char **argv) {

  ros::init(argc, argv, "trajectory_node");
  ros::NodeHandle nh;

  if (!check_realtime())
    throw std::runtime_error("REALTIME NOT AVAILABLE");

  if (!set_realtime_SCHED_FIFO())
    throw std::runtime_error("ERROR IN set_realtime_SCHED_FIFO");

  ros::Subscriber state_sub =
      nh.subscribe<sensor_msgs::JointState>("joint_state", 1, stateCallBack);
  actionlib::SimpleActionClient<pick_and_place_libfranka::JointTrajectoryAction>
      joint_traj_ac("joint_traj", true);

  ros::Rate slow_loop(100);
  while (ros::ok() && !state_read) {
    ros::spinOnce();
    slow_loop.sleep();
  }

  std::array<double, 7> q_init = current_state;
  std::array<double, 7> q_goal{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};

  double t = 0.0;    // s
  double Ts = 0.001; // s
  double Tf = 10.0;  // s

  /** JOINT TRAJECTORY ACTION **/

  pick_and_place_libfranka::JointTrajectoryGoal joint_traj_goal;
  joint_traj_goal.trajectory.joint_names = {"j1", "j2", "j3", "j4",
                                            "j5", "j6", "j7"};
  trajectory_msgs::JointTrajectoryPoint point;

  while (ros::ok() && t <= Tf) {

    double tau = t / Tf; // asse dei tempi normalizzato
    point.positions.clear();

    // Costruzione del messaggio che invia il comando q(t) = q_init + (q_goal -
    // q_init)*q(t/tf)
    for (int i = 0; i < 7; i++) {
      point.positions.push_back(
          q_init[i] +
          (q_goal[i] - q_init[i]) *
              (6 * pow(tau, 5) - 15 * pow(tau, 4) + 10 * pow(tau, 3)));
    }
    point.time_from_start = ros::Duration(t);
    joint_traj_goal.trajectory.points.push_back(point);

    t = t + Ts; // tempo trascorso
  }
  std::cout << "Trajectory created, waiting for server then sending goal \n";
  joint_traj_ac.waitForServer();

  joint_traj_ac.sendGoal(joint_traj_goal);

  bool finished_before_timeout =
      joint_traj_ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout) {
    pick_and_place_libfranka::JointTrajectoryResultConstPtr result =
        joint_traj_ac.getResult();
    ROS_INFO("Action finished, success :  %d", result->success);
  } else
    ROS_INFO("Action did not finish before the time out.");

  /******** CARTESIAN TRAJECTORY ACTION ***********/

  // TooN::Matrix<4, 4, double> n_T_e(TooN::Data(0.7071, 0.7071, 0.0, 0.0,
  //                                             -0.7071,
  //                                             0.7071, 0.0, 0.0, 0.0,
  //                                             0.0, 1.0, 0.1034, 0.0, 0.0,
  //                                             0.0, 1.0));
  // Quando viene montato il gripper assicurarsi che pz sia impostato su AppDesk
  // a 0.1034 prima di utilizzare questa matrice di trasformazione

  actionlib::SimpleActionClient<
      pick_and_place_libfranka::CartesianTrajectoryAction>
      cartesian_traj_ac("cartesian_traj", true);

  TooN::Matrix<4, 4, double> n_T_e = TooN::Identity(4);
  sun::Panda panda(n_T_e, 1.0, "panda");

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

  // Desired Pose
  TooN::Vector<3, double> pf = pi + TooN::makeVector(0.01, 0.0, 0.0);

  TooN::Matrix<3, 3> Rot_des = TooN::Data(1, 0, 0, 0, 0, 1, 0, -1, 0);
  sun::UnitQuaternion final_quat(Rot_des); // desired quaternion
  sun::UnitQuaternion delta_quat = final_quat * inv(init_quat);
  sun::AngVec angvec = delta_quat.toangvec();

  Tf = 10.0;
  sun::Quintic_Poly_Traj qp_position(
      Tf, 0.0, 1.0); // polinomio quintico utilizzato per line_traj
  sun::Quintic_Poly_Traj qp_orientation(
      Tf, 0.0, angvec.getAng()); // polinomio quintico utilizzato per quat_traj

  sun::Line_Segment_Traj line_traj(pi, pf, qp_position);
  sun::Rotation_Const_Axis_Traj quat_traj(init_quat, angvec.getVec(),
                                          qp_orientation);

  // Nota: la traiettoria in orientamento è quella definita dal Delta_quat.
  // Perchè vogliamo andare da init_quat (orientamento iniziale) a final_quat
  // (orientamento finale). Il metodo getquaternion(t) restituisce DeltaQuat(t)
  // * init_quat. Il metodo getPosition(t) resituisce direttamente la posizione
  // corretta tra pi e pf.

  sun::Cartesian_Independent_Traj cartesian_traj(line_traj, quat_traj);
  pick_and_place_libfranka::CartesianTrajectoryGoal cartesian_traj_goal;
  Ts = 0.001;
  cartesian_traj_goal.trajectory.points.reserve(Tf / Ts);

  trajectory_msgs::MultiDOFJointTrajectoryPoint cartesian_point;

  TooN::Vector<3, double> posizione_d;
  sun::UnitQuaternion unit_quat_d;
  while (ros::ok() && !cartesian_traj.isCompleate(t)) {

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

    t = t + Ts; // tempo trascorso
  }

  std::cout << "Cartesian Trajectory created, waiting for server then sending "
               "goal \n";
  cartesian_traj_ac.waitForServer();

  cartesian_traj_ac.sendGoal(cartesian_traj_goal);

  finished_before_timeout =
      cartesian_traj_ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout) {
    pick_and_place_libfranka::CartesianTrajectoryResultConstPtr result =
        cartesian_traj_ac.getResult();
    ROS_INFO("Action finished, success :  %d", result->success);
  } else
    ROS_INFO("Action did not finish before the time out.");

  return 0;
}

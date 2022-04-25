
#include <pick_and_place_libfranka/trajectory_node.h>

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
  {
    pick_and_place_libfranka::JointTrajectoryGoal joint_traj_goal;
    joint_traj_goal.trajectory.joint_names = {"j1", "j2", "j3", "j4",
                                              "j5", "j6", "j7"};
    trajectory_msgs::JointTrajectoryPoint point;

    while (ros::ok() && t <= Tf) {

      double tau = t / Tf; // asse dei tempi normalizzato
      point.positions.clear();

      // Costruzione del messaggio che invia il comando q(t) = q_init + (q_goal
      // - q_init)*q(t/tf)
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
  }

  /******** CARTESIAN TRAJECTORY ACTION ***********/
  {
    pick_and_place_libfranka::CartesianTrajectoryGoal cartesian_traj_goal;

    actionlib::SimpleActionClient<
        pick_and_place_libfranka::CartesianTrajectoryAction>
        cartesian_traj_ac("cartesian_traj", true);
    cartesian_traj_ac.waitForServer();

    // TooN::Matrix<4, 4, double> n_T_e(TooN::Data(0.7071, 0.7071, 0.0, 0.0,
    //                                             -0.7071,
    //                                             0.7071, 0.0, 0.0, 0.0,
    //                                             0.0, 1.0, 0.1034, 0.0, 0.0,
    //                                             0.0, 1.0));
    // Quando viene montato il gripper assicurarsi che pz sia impostato su
    // AppDesk a 0.1034 prima di utilizzare questa matrice di trasformazione
    TooN::Matrix<4, 4, double> n_T_e = TooN::Identity(4);
    sun::Panda panda(n_T_e, 1.0, "panda");

    CartesianPoint final_pose;
    TooN::Matrix<3, 3, double> R_ortogonal =
        TooN::Data(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
    final_pose.position = TooN::makeVector(0.5, 0.2, 0.3);
    final_pose.quaternion =
        sun::UnitQuaternion(R_ortogonal * sun::roty(30.0 * M_PI / 180.0));
    final_pose.Tf = 10;

    cartesian_traj_goal = CartesianPoint2CartesianTrajGoal(final_pose, panda);

    cartesian_traj_ac.sendGoal(cartesian_traj_goal);

    bool finished_before_timeout =
        cartesian_traj_ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
      pick_and_place_libfranka::CartesianTrajectoryResultConstPtr result =
          cartesian_traj_ac.getResult();
      ROS_INFO("Action finished, success :  %d", result->success);
    } else
      ROS_INFO("Action did not finish before the time out.");
  }

  return 0;
}

#include "pick_and_place_libfranka/TrajAction.h"
#include "pick_and_place_libfranka/JointTrajectoryHelper.h"

void TrajAction::publish_state() {

  try {

    sensor_msgs::JointState msg;
    msg.name = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};
    std::array<double, 7UL> joint_state{};

    while (ros::ok()) {

      msg.header.stamp = ros::Time::now();

      msg.position.clear();

      // Area critica
      robot_mutex_.lock();
      joint_state = robot_->readOnce().q;
      robot_mutex_.unlock();

      for (int i = 0; i < 7; i++)
        msg.position.push_back(joint_state[i]);

      state_pub_.publish(msg);
    }

  } catch (const franka::Exception &ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
}

TrajAction::TrajAction()
    : joint_point_traj_as_(nh_, "joint_point_traj",
                           boost::bind(&TrajAction::JointPointTrajCB, this, _1),
                           false),
      joint_traj_as_(nh_, "joint_traj",
                     boost::bind(&TrajAction::JointTrajCB, this, _1), false) {

  std::string robot_IP;
  if (!nh_.getParam("robot_ip", robot_IP)) {
    ROS_ERROR_STREAM("Specificare l'indirizzo IP del robot.");
  }

  try {

    // Connessione al robot
    robot_ = std::make_unique<franka::Robot>(robot_IP);

    setDefaultBehavior(*robot_);

    state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 1);
    // start publisher thread
    publish_thread_ = std::make_unique<std::thread>(
        std::bind(&TrajAction::publish_state, this));

    // Start as
    joint_point_traj_as_.start();
    joint_traj_as_.start();

    std::cout << "Creato oggetto ActionTraj e avviati i server \n";

  } catch (const franka::Exception &ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
}

TrajAction::~TrajAction(void) { publish_thread_->join(); }

void TrajAction::JointTrajCB(
    const pick_and_place_libfranka::JointTrajectoryGoalConstPtr &goal) {

  using std::pow;
  pick_and_place_libfranka::JointTrajectoryFeedback joint_traj_feedback;
  pick_and_place_libfranka::JointTrajectoryResult joint_traj_result;

  try {

    robot_mutex_.lock();

    // get initial state and add it to the trajectory
    trajectory_msgs::JointTrajectoryPoint initial_state;
    for (int i = 0; i < 7; i++) {
      initial_state.positions.push_back(robot_->readOnce().q[i]);
    }
    initial_state.time_from_start = ros::Duration(0.0);

    std::vector<trajectory_msgs::JointTrajectoryPoint> points;
    points.push_back(initial_state);
    for (int i = 0; i < goal->trajectory.points.size(); i++)
      points.push_back(goal->trajectory.points[i]);

    // Interpolation
    std::vector<TooN::Matrix<TooN::Dynamic, 4, double>> coeff =
        compute_polynomial_interpolation(points);

    double tf =
        points.back().time_from_start.toSec(); // desired trajectory duration
    joint_traj_feedback.time_left = tf;
    joint_traj_as_.publishFeedback(joint_traj_feedback);

    sensor_msgs::JointState joint_state;
    joint_state.name = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};

    // Start executing trajectory
    std::array<double, 7> q_command;
    double t = 0.0;
    int p = 0;
    bool success = false;

    robot_->control([&](const franka::RobotState &robot_state,
                        franka::Duration period) -> franka::JointPositions {
      // Publishing current joint_state
      joint_state.position.clear();
      for (int i = 0; i < 7; i++) {
        joint_state.position.push_back(robot_state.q[i]);
      }
      joint_state.header.stamp = ros::Time::now();
      state_pub_.publish(joint_state);

      // track passing of time
      t += period.toSec();
      joint_traj_feedback.time_left = tf - t;
      joint_traj_as_.publishFeedback(joint_traj_feedback);

      // Compute desired joint state
      for (int joint = 0; joint < 7; joint++)
        q_command[joint] = coeff[joint](p, 0) + coeff[joint](p, 1) * t +
                           coeff[joint](p, 2) * pow(t, 2) +
                           coeff[joint](p, 3) * pow(t, 3);

      // Check if the action has been preempteed
      if (joint_traj_as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("Preempted: Joint Trajectory \n ");
        // set the action state to preempted
        joint_point_traj_as_.setPreempted();
        return franka::MotionFinished(franka::JointPositions(q_command));
      }

      // Change polynomial coefficients.
      if (t > points[p + 1].time_from_start.toSec())
        p++;

      if (t < tf)
        return franka::JointPositions(q_command);
      else {
        std::cout << "Motion finished \n";
        success = true;
        return franka::MotionFinished(franka::JointPositions(q_command));
      }
    });

    if (success) {
      joint_traj_result.success = true;
      ROS_INFO("Succeeded: Joint Trajectory \n ");
      // set the action state to succeeded
      joint_traj_as_.setSucceeded(joint_traj_result);
    }

  } catch (const franka::Exception &ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  robot_mutex_.unlock();
}

void TrajAction::JointPointTrajCB(
    const pick_and_place_libfranka::JointPointTrajectoryGoalConstPtr &goal) {

  pick_and_place_libfranka::JointPointTrajectoryFeedback joint_point_feedback;
  pick_and_place_libfranka::JointPointTrajectoryResult joint_point_result;

  robot_mutex_.lock();

  try {

    // Modello cinematico e dinamico del robot
    franka::Model model = robot_->loadModel();

    // Lettura dello stato attuale del robot
    franka::RobotState initial_state;
    std::array<double, 7> q_init_{};

    do {
      initial_state = robot_->readOnce();
      q_init_ = initial_state.q;
    } while (q_init_[3] ==
             0.0); // La quarta variabile di giunto è sempre negativa

    // Doc:
    // https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html#a5b5ba0a4f2bfd20be963b05622e629e1

    joint_point_feedback.time_left = goal->desired_tf;
    std::array<double, 7> q_command;
    std::array<double, 7> q_goal;
    for (int i = 0; i < 7; i++)
      q_goal[i] = goal->desired_conf[i];

    bool success = false;
    double t = 0;
    double tf = goal->desired_tf; // s

    sensor_msgs::JointState joint_state;
    joint_state.name = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};

    robot_->control([&](const franka::RobotState &robot_state,
                        franka::Duration period) -> franka::JointPositions {
      // Publishing joint_state
      joint_state.position.clear();
      for (int i = 0; i < 7; i++) {
        joint_state.position.push_back(robot_state.q[i]);
      }
      joint_state.header.stamp = ros::Time::now();
      state_pub_.publish(joint_state);

      t += period.toSec();
      double tau = t / tf;
      for (int i = 0; i < 7; i++)
        q_command[i] = q_init_[i] + (q_goal[i] - q_init_[i]) *
                                        (6 * pow(tau, 5) - 15 * pow(tau, 4) +
                                         10 * pow(tau, 3));

      if (joint_point_traj_as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("Preempted: Joint Point Trajectory \n ");
        // set the action state to preempted
        joint_point_traj_as_.setPreempted();
        return franka::MotionFinished(franka::JointPositions(q_command));
      }

      joint_point_feedback.time_left = goal->desired_tf - t;
      joint_point_traj_as_.publishFeedback(joint_point_feedback);

      if (t < tf) {
        return franka::JointPositions(q_command);
      } else {
        std::cout << "Motion finished \n";
        success = true;
        return franka::MotionFinished(franka::JointPositions(q_goal));
      }
    });

    if (success) {
      joint_point_result.success = true;
      ROS_INFO("Succeeded: Joint Point Trajectory \n ");
      // set the action state to succeeded
      joint_point_traj_as_.setSucceeded(joint_point_result);
    }

  } catch (const franka::Exception &ex) {

    // print exception
    std::cout << ex.what() << std::endl;
  }

  robot_mutex_.unlock();
}

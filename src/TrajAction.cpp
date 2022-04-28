#include "pick_and_place_libfranka/TrajAction.h"

void TrajAction::publish_state() {

  try {

    sensor_msgs::JointState msg;
    msg.name = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};
    std::array<double, 7UL> joint_state{};
    ros::Rate loop_rate(1000);
    while (ros::ok()) {

      msg.header.stamp = ros::Time::now();

      msg.position.clear();

      // Area critica
      robot_mutex_->lock();

      joint_state = robot_->readOnce().q;

      robot_mutex_->unlock();

      for (int i = 0; i < 7; i++)
        msg.position.push_back(joint_state[i]);

      state_pub_.publish(msg);

      loop_rate.sleep();
    }
  } catch (const franka::Exception &ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
}

TrajAction::TrajAction(std::shared_ptr<franka::Robot> panda,
                       std::mutex *robot_mutex)
    : joint_point_traj_as_(nh_, "joint_point_traj",
                           boost::bind(&TrajAction::JointPointTrajCB, this, _1),
                           false),
      joint_traj_as_(nh_, "joint_traj",
                     boost::bind(&TrajAction::JointTrajCB, this, _1), false),
      cartesian_traj_as_(nh_, "cartesian_traj",
                         boost::bind(&TrajAction::CartesianTrajCB, this, _1),
                         false) {

  if (!nh_.getParam("publish_command", publish_command)) {
    ROS_ERROR_STREAM("Param publish_command not found.");
  }

  if (!nh_.getParam("debugging", debugging)) {
    ROS_ERROR_STREAM("Param debugging not found.");
  }

  try {

    // Connect to robot
    robot_ = panda;

    // Robot mutex
    robot_mutex_ = robot_mutex;

    setDefaultBehavior(*robot_);

    state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 1);
    // Start publisher thread
    publish_thread_ = std::make_unique<std::thread>(
        std::bind(&TrajAction::publish_state, this));

    // Start action servers
    joint_point_traj_as_.start();
    joint_traj_as_.start();
    cartesian_traj_as_.start();

    std::cout << "Creato oggetto ActionTraj e avviati i server \n";
  } catch (const franka::Exception &ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
}

TrajAction::~TrajAction(void) { publish_thread_->join(); }

void TrajAction::JointPointTrajCB(
    const pick_and_place_libfranka::JointPointTrajectoryGoalConstPtr &goal) {

  pick_and_place_libfranka::JointPointTrajectoryFeedback joint_point_feedback;
  pick_and_place_libfranka::JointPointTrajectoryResult joint_point_result;

  robot_mutex_->lock();
  ros::NodeHandle nh;
  ros::Publisher command_publisher =
      nh.advertise<sensor_msgs::JointState>("joint_command", 1);

  try {

    // Modello cinematico e dinamico del robot
    franka::Model model = robot_->loadModel();

    // Lettura dello stato attuale del robot
    franka::RobotState initial_state;
    std::array<double, 7> q_init_{};

    do {
      initial_state = robot_->readOnce();
      q_init_ = initial_state.q_d;
    } while (q_init_[3] ==
             0.0); // La quarta variabile di giunto è sempre negativa

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
    sensor_msgs::JointState joint_command;
    joint_command.name = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};

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
        // Evitare il motion finished diretto (robot is still moving)
        return franka::MotionFinished(franka::JointPositions(q_command));
      }

      // Publish joint command
      if (publish_command) {
        joint_command.position.clear();
        for (int i = 0; i < 7; i++) {
          joint_command.position.push_back(q_command[i]);
        }
        joint_command.header.stamp = ros::Time::now();
        command_publisher.publish(joint_command);
      }

      joint_point_feedback.time_left = goal->desired_tf - t;
      joint_point_traj_as_.publishFeedback(joint_point_feedback);

      if (t < tf)
        return debugging ? franka::JointPositions(q_init_)
                         : franka::JointPositions(q_command);
      else {
        std::cout << "Joint Point Traj action finished \n";
        success = true;
        return debugging
                   ? franka::MotionFinished(franka::JointPositions(q_init_))
                   : franka::MotionFinished(franka::JointPositions(q_command));
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

  robot_mutex_->unlock();
}

/** If the given trajectory has a starting point with time_from_start = 0 it
 * gets deleted. The first configuration of the trajectory will always be the
 * current configuration.
 **/
void TrajAction::JointTrajCB(
    const pick_and_place_libfranka::JointTrajectoryGoalConstPtr &goal) {

  using std::pow;
  pick_and_place_libfranka::JointTrajectoryFeedback joint_traj_feedback;
  pick_and_place_libfranka::JointTrajectoryResult joint_traj_result;
  ros::NodeHandle nh;
  ros::Publisher command_publisher =
      nh.advertise<sensor_msgs::JointState>("joint_command", 1);

  try {

    robot_mutex_->lock();
    std::vector<trajectory_msgs::JointTrajectoryPoint> points;

    // Note: q_d (desired) not q (measured)
    std::array<double, 7> initial_conf_std = robot_->readOnce().q_d;

    // Extract the trajectory from the goal and build the vector
    for (long unsigned int i = 0; i < goal->trajectory.points.size(); i++)
      points.push_back(goal->trajectory.points[i]);

    // Add intial configuration as starting point
    trajectory_msgs::JointTrajectoryPoint initial_conf;
    initial_conf.time_from_start = ros::Duration(0.0);
    for (int i = 0; i < 7; i++)
      initial_conf.positions.push_back(initial_conf_std[i]);

    // If the desired trajectory defines a starting point just cancel it
    if (points.front().time_from_start.toSec() == 0.0) {
      points.erase(points.begin());
    }

    points.insert(points.begin(), initial_conf);

    // 3rd degree interpolation of the given points
    std::vector<TooN::Matrix<TooN::Dynamic, 4, double>> coeff =
        compute_polynomial_interpolation(points);

    // Publish feedback
    double tf =
        points.back()
            .time_from_start.toSec(); // desired duration of the trajectory

    joint_traj_feedback.time_left = tf;
    joint_traj_as_.publishFeedback(joint_traj_feedback);

    // Publishing variables
    sensor_msgs::JointState joint_state;
    joint_state.name = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};
    sensor_msgs::JointState joint_command;
    joint_command.name = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};

    // Start executing trajectory
    std::array<double, 7> q_command;
    double t = 0.0;
    int p = 0; // tracks the polynomial coefficients that have to be used
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
      t += period.toSec(); // first time period == 0.0
      joint_traj_feedback.time_left = tf - t;
      joint_traj_as_.publishFeedback(joint_traj_feedback);

      // Compute desired joint state
      for (int joint = 0; joint < 7; joint++)
        q_command[joint] = coeff[joint](p, 0) + coeff[joint](p, 1) * t +
                           coeff[joint](p, 2) * pow(t, 2) +
                           coeff[joint](p, 3) * pow(t, 3);

      // Publish joint command
      if (publish_command) {
        joint_command.position.clear();
        for (int i = 0; i < 7; i++) {
          joint_command.position.push_back(q_command[i]);
        }
        joint_command.header.stamp = ros::Time::now();
        command_publisher.publish(joint_command);
      }

      // Check if the action has been preempteed
      if (joint_traj_as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("Preempted: Joint Trajectory \n ");
        // set the action state to preempted
        joint_traj_as_.setPreempted();
        // Evitare il motion finished diretto (robot is still moving)
        return franka::MotionFinished(franka::JointPositions(q_command));
      }

      // Change polynomial coefficients / point
      if (t > points[p + 1].time_from_start.toSec())
        p++;

      if (t < tf)
        return debugging ? franka::JointPositions(initial_conf_std)
                         : franka::JointPositions(q_command);
      else {
        std::cout << "Joint Traj action finished \n";
        success = true;
        return debugging
                   ? franka::MotionFinished(
                         franka::JointPositions(initial_conf_std))
                   : franka::MotionFinished(franka::JointPositions(q_command));
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
    joint_traj_result.success = false;
    joint_traj_as_.setSucceeded(joint_traj_result);
  }

  robot_mutex_->unlock();
}

/** If the given trajectory has a starting point with time_from_start = 0 it
 * gets deleted. The first configuration of the trajectory will always be the
 * current configuration.
 **/
void TrajAction::CartesianTrajCB(
    const pick_and_place_libfranka::CartesianTrajectoryGoalConstPtr &goal) {
  using std::pow;
  pick_and_place_libfranka::CartesianTrajectoryFeedback cartesian_traj_feedback;
  pick_and_place_libfranka::CartesianTrajectoryResult cartesian_result;
  ros::NodeHandle nh;
  ros::Publisher command_publisher =
      nh.advertise<sensor_msgs::JointState>("joint_command", 1);
  std::array<double, 7> final_command;

  try {
    robot_mutex_->lock();

    // Extract cartesian trajectory
    std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> points;
    for (long unsigned int i = 0; i < goal->trajectory.points.size(); i++)
      points.push_back(goal->trajectory.points[i]);

    std::vector<trajectory_msgs::JointTrajectoryPoint> joint_trajectory;
    std::array<double, 7> initial_configuration =
        robot_->readOnce().q_d; // q_d instead of q. If debugging is on then qd
                                // will be different from q

    // Compute Joint trajectory with inverse kinematics
    // The initial configuration will be added as first point of the trajectory
    TooN::Matrix<4, 4, double> n_T_e = TooN::Identity(4); // NO HAND attached
    double Ts = 0.001;
    joint_trajectory = panda_clik(points, initial_configuration, n_T_e, Ts);

    // 3rd degree interpolation of the given points

    std::vector<TooN::Matrix<TooN::Dynamic, 4, double>> coeff =
        compute_polynomial_interpolation(joint_trajectory);

    // Publish feedback
    double tf = joint_trajectory.back().time_from_start.toSec(); // s
    cartesian_traj_feedback.time_left = tf;
    cartesian_traj_as_.publishFeedback(cartesian_traj_feedback);

    // Publishing variables
    sensor_msgs::JointState joint_state;
    joint_state.name = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};
    sensor_msgs::JointState joint_command;
    joint_command.name = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};

    // Start executing traj
    std::array<double, 7> q_command;
    double t = 0.0;
    int p = 0; // tracks the polynomial coefficients that have to be used
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

      // Track passing of time
      t += period.toSec(); // first time period == 0.0
      cartesian_traj_feedback.time_left = tf - t;
      cartesian_traj_as_.publishFeedback(cartesian_traj_feedback);

      // Compute desired joint state
      for (int joint = 0; joint < 7; joint++)
        q_command[joint] = coeff[joint](p, 0) + coeff[joint](p, 1) * t +
                           coeff[joint](p, 2) * pow(t, 2) +
                           coeff[joint](p, 3) * pow(t, 3);

      // Publish joint command
      if (publish_command) {
        joint_command.position.clear();
        for (int i = 0; i < 7; i++) {
          joint_command.position.push_back(q_command[i]);
        }
        joint_command.header.stamp = ros::Time::now();
        command_publisher.publish(joint_command);
      }

      // Check if the action has been preempteed
      if (cartesian_traj_as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("Preempted: Joint Trajectory \n ");
        // set the action state to preempted
        cartesian_traj_as_.setPreempted();
        // Evitare il motion finished diretto (robot is still moving)
        return franka::MotionFinished(franka::JointPositions(q_command));
      }

      // Change polynomial coefficients
      if (t > points[p + 1].time_from_start.toSec())
        p++;

      if (t < tf) {

        return debugging ? franka::JointPositions(initial_configuration)
                         : franka::JointPositions(q_command);
      }

      else {

        std::cout << "Cartesian Traj action finished \n";
        success = true;
        return debugging
                   ? franka::MotionFinished(
                         franka::JointPositions(initial_configuration))
                   : franka::MotionFinished(franka::JointPositions(q_command));
      }
    });

    if (success) {
      cartesian_result.success = true;
      ROS_INFO("Succeeded: Cartesian Point Trajectory \n ");
      // set the action state to succeeded
      cartesian_traj_as_.setSucceeded(cartesian_result);
    }
  } catch (const franka::Exception &ex) {
    // print exception
    cartesian_result.success = false;
    cartesian_traj_as_.setSucceeded(cartesian_result);
    std::cout << ex.what() << std::endl;
  }

  robot_mutex_->unlock();
}
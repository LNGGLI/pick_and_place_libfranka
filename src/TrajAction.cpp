#include "pick_and_place_libfranka/TrajAction.h"

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

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
    // robot_ = std::make_unique<franka::Robot>(robot_IP);

    // setDefaultBehavior(*robot_);

    state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 1);
    // start publisher thread
    // publish_thread_ = std::make_unique<std::thread>(
    //     std::bind(&TrajAction::publish_state, this));

    // Start as
    // joint_point_traj_as_.start();
    joint_traj_as_.start();

    std::cout << "Creato oggetto ActionTraj e avviati i server \n";

  } catch (const franka::Exception &ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
}

TrajAction::~TrajAction(void) { publish_thread_->join(); }

TooN::Vector<TooN::Dynamic, double> compute_qdot_vector(
    const std::vector<trajectory_msgs::JointTrajectoryPoint> &points,
    const int &joint) {

  int n_points = points.size();
  TooN::Vector<TooN::Dynamic, double> qdot(n_points);
  qdot[0] = 0.0;
  qdot[n_points - 1] = 0.0;
  double vk = 0.0;
  double vk_1 = 0.0;

  vk = (points[1].positions[joint] - points[0].positions[joint]) /
       (points[1].time_from_start.toSec() - points[0].time_from_start.toSec());

  for (int k = 1; k < n_points - 1; k++) { // 0 e n-1 already assigned

    vk_1 = (points[k + 1].positions[joint] - points[k].positions[joint]) /
           (points[k + 1].time_from_start.toSec() -
            points[k].time_from_start.toSec());

    qdot[k] = sgn(vk) != sgn(vk_1) ? 0.0 : 0.5 * (vk + vk_1);

    vk = vk_1;
  }

  return qdot;
}

/**
 * @brief Computes the coefficents of the polynomial functions (3rd degree) that
 * interpolate the given points for the given joint.
 *
 * @param points the full joint position trajectory of type
 * std::vector<trajectory_msgs::JointTrajectoryPoint>
 * @param joint joint whose trajectory you want to interpolate
 *
 * @return ** TooN::Matrix<TooN::Dynamic, 4, double> with poly coefficents
 */
TooN::Matrix<TooN::Dynamic, 4, double> compute_poly_coefficients(
    const std::vector<trajectory_msgs::JointTrajectoryPoint> &points,
    int joint) {

  int n_poly = points.size() - 1; // number of polynomial functions used
  TooN::Vector<TooN::Dynamic, double> qdot_vec =
      compute_qdot_vector(points, joint); // Controllata

  TooN::Matrix<TooN::Dynamic, 4, double> result(n_poly, 4); // coeff polinomi
  TooN::Vector<4, double> b;                                // termine noto
  TooN::Matrix<4, 4, double> A;

  for (int k = 0; k < n_poly; k++) {

    // fill b
    b[0] = points[k].positions[joint];     // Pk (tk)
    b[1] = points[k + 1].positions[joint]; // Pk (tk+1)
    b[2] = qdot_vec[k];                    // Pk_dot(tk)
    b[3] = qdot_vec[k + 1];                // Pk_dot(tk+1)

    // fill A
    double tk = points[k].time_from_start.toSec();
    double tk_1 = points[k + 1].time_from_start.toSec();

    using std::pow;
    for (int j = 0; j < 4; j++) {
      A[0][j] = pow(tk, j);
      A[1][j] = pow(tk_1, j);
    }

    A[2][0] = 0.0;
    A[2][1] = 1.0;
    A[2][2] = 2 * pow(tk, 1);
    A[2][3] = 3 * pow(tk, 2);
    A[3][0] = 0.0;
    A[3][1] = 1.0;
    A[3][2] = 2 * pow(tk_1, 1);
    A[3][3] = 3 * pow(tk_1, 2);

    // Solve system
    result[k] = TooN::gaussian_elimination(A, b);
    std::cout << " For joint " << joint << " coeff poly " << k << "= "
              << result[k] << "\n";
  }

  return result;
}

/**
 * @brief Computes the coefficients of the polynomial functions used to
 * interpolate the given points. It uses 3rd degree polynomial functions and
 * the velocity at each path point is computed such that the robot doesn't
 * stop at each point.
 * @param points: full trajectory as a
 * std::vector<trajectory_msgs::JointTrajectoryPoint>
 *
 * @return returns a 7 elements std::vector of toon matrices (one for each
 * joint). Each toon matrix is a N-1 x 4 where N is the number of given
 * points. The coefficients go from a0 to a3
 *
 */
std::vector<TooN::Matrix<TooN::Dynamic, 4, double>> // return type
compute_polynomial_interpolation(
    const std::vector<trajectory_msgs::JointTrajectoryPoint> &points) {

  TooN::Matrix<TooN::Dynamic, 4, double> poly(points.size() - 1, 4);
  std::vector<TooN::Matrix<TooN::Dynamic, 4, double>> polys;

  for (int joint = 0; joint < 7; joint++) {
    poly = compute_poly_coefficients(points, joint);
    polys.push_back(poly); // matrices be one next to the other: 4 x 7*(N-1)
  }

  return polys;
}

void TrajAction::JointTrajCB(
    const pick_and_place_libfranka::JointTrajectoryGoalConstPtr &goal) {

  using std::pow;
  pick_and_place_libfranka::JointTrajectoryFeedback joint_traj_feedback;
  pick_and_place_libfranka::JointTrajectoryResult joint_traj_result_;

  try {

    robot_mutex_.lock();

    // Interpolation
    trajectory_msgs::JointTrajectoryPoint initial_state;
    for (int i = 0; i < 7; i++) {
      initial_state.positions.push_back(0.0);
    }
    initial_state.time_from_start = ros::Duration(0.0);

    std::vector<trajectory_msgs::JointTrajectoryPoint> points;
    points.push_back(initial_state);
    for (int i = 0; i < goal->trajectory.points.size(); i++)
      points.push_back(goal->trajectory.points[i]);

    double tf = points.back().time_from_start.toSec();

    std::vector<TooN::Matrix<TooN::Dynamic, 4, double>> coeff =
        compute_polynomial_interpolation(points);

    for (int joint = 0; joint < 7; joint++) {
      std::cout << "Matrice dei coefficienti giunto " << joint << "\n";
      for (int i = 0; i < points.size() - 1; i++) {
        for (int j = 0; j < 4; j++) {
          std::cout << coeff[joint][i][j] << " ";
        }
        std::cout << "\n\n";
      }
    }

    ROS_INFO("\nComputed polynomial interpolation\n");
    joint_traj_feedback.time_left = tf;
    joint_traj_as_.publishFeedback(joint_traj_feedback);

    // Start executing trajectory
    std::array<double, 7> q_command;

    double t = 0;
    ros::Rate loop_rate(1000);
    int p = 0;
    while (ros::ok() && t < tf) {

      t = t + 0.001;
      joint_traj_feedback.time_left = tf - t;
      joint_traj_as_.publishFeedback(joint_traj_feedback);

      // Compute q_command (3rd degree polynomial function)
      for (int i = 0; i < 7; i++)
        q_command[i] = coeff[i](p, 0) + coeff[i](p, 1) * t +
                       coeff[i](p, 2) * pow(t, 2) + coeff[i](p, 3) * pow(t, 3);

      // Publishing joint_state
      sensor_msgs::JointState joint_state;
      joint_state.name = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};
      for (int i = 0; i < 7; i++) {
        joint_state.position.push_back(q_command[i]);
      }
      joint_state.header.stamp = ros::Time::now();
      state_pub_.publish(joint_state);

      // Change polynomial function: must be the last thing in the cycle
      if (t > points[p + 1].time_from_start.toSec()) {
        p++;
        ROS_INFO("Switched polynomial function, t = %f \n", t);
      }
      loop_rate.sleep();
    }

    joint_traj_result_.success = true;
    joint_traj_as_.setSucceeded(joint_traj_result_);

    // robot_->control([&](const franka::RobotState &,
    //                     franka::Duration period) -> franka::JointPositions {
    //   t += period.toSec();

    //   return franka::JointPositions(q_c);
    // });

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

      // Publishing joint_state
      joint_state.position.clear();
      for (int i = 0; i < 7; i++) {
        joint_state.position.push_back(robot_state.q[i]);
      }
      joint_state.header.stamp = ros::Time::now();
      state_pub_.publish(joint_state);
    });

    if (success) {
      joint_point_result.success = true;
      ROS_INFO("Succeeded: Joint Point Trajectory \n ");
      // set the action state to succeeded
      joint_point_traj_as_.setSucceeded(joint_point_result);
    }

  } catch (const franka::Exception &ex) {
    robot_mutex_.unlock();
    // print exception
    std::cout << ex.what() << std::endl;
  }
}

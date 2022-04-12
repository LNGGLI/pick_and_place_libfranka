#include "pick_and_place_libfranka/TrajAction.h"



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
    std::cout << ex.what()  << std::endl;
  }
}


TrajAction::TrajAction()
    : joint_point_traj_as_(nh_, "joint_point_traj",
                           boost::bind(&TrajAction::JointPointTrajCB, this, _1), false)
{

  
  std::string robot_IP;
  if (!nh_.getParam("robot_ip", robot_IP)) {
    ROS_ERROR_STREAM("Specificare l'indirizzo IP del robot.");
  }
  try {

    // Connessione al robot
    robot_ = std::make_unique<franka::Robot>(robot_IP);
    // setDefaultBehavior(*robot_);
    
    state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state",1);
    // start publisher thread
    publish_thread_ = std::make_unique<std::thread>(
        std::bind(&TrajAction::publish_state, this));

    // Start as
    joint_point_traj_as_.start();

    std::cout << "Creato oggetto ActionTraj e avviato il server";

  } catch (const franka::Exception &ex) {
    // print exception
    std::cout << ex.what()  << std::endl;
  }
}

TrajAction::~TrajAction(void) {
  publish_thread_->join();
}

void TrajAction::JointPointTrajCB(
    const pick_and_place_libfranka::JointPointTrajectoryGoalConstPtr &goal) {


  try {

    robot_mutex_.lock();
    // Modello cinematico e dinamico del robot
    franka::Model model = robot_->loadModel();

    // Lettura dello stato attuale del robot
    franka::RobotState initial_state;
    std::array<double, 7> q_init_{};

    do {
      initial_state = robot_->readOnce();
      q_init_ = initial_state.q;
    } while (q_init_[3] == 0.0); // La quarta variabile di giunto è sempre negativa

    // Doc:
    // https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html#a5b5ba0a4f2bfd20be963b05622e629e1

    feedback_.time_left = goal->desired_tf;
    std::array<double, 7> q_command;
    std::array<double, 7> q_goal;
    for (int i = 0; i < 7; i++)
      q_goal[i] = goal->desired_conf[i];

    bool success = false;
    double t = 0;
    double tf = goal->desired_tf; // s

    robot_->control([&t, tf, q_init_, q_goal, goal, &success, &q_command,
                    this](const franka::RobotState &,
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

      this->feedback_.time_left = goal->desired_tf - t;
      joint_point_traj_as_.publishFeedback(feedback_);

      if (t < tf) {
        return franka::JointPositions(q_command);
      } else {
        std::cout << "Motion finished \n";
        success = true;
        return franka::MotionFinished(franka::JointPositions(q_command));
      }
    });

    if (success) {
      result_.success = true;
      ROS_INFO("Succeeded: Joint Point Trajectory \n ");
      // set the action state to succeeded
      joint_point_traj_as_.setSucceeded(result_);
    }

    robot_mutex_.unlock();


  } catch (const franka::Exception &ex) {
    robot_mutex_.unlock();
    // print exception
    std::cout << ex.what() << std::endl;
  }
}

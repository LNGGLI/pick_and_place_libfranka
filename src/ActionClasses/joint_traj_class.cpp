

#include <pick_and_place_libfranka/ActionClasses/action_classes.h>

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
#include <pick_and_place_libfranka/JointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

JointTrajAction::JointTrajAction(std::string name)
    : as_(nh_, name, boost::bind(&JointTrajAction::executeCB, this, _1), false),
      action_name_(name) {

  as_.start(); 
}

JointTrajAction::~JointTrajAction(void) {}

void JointTrajAction::executeCB(
    const pick_and_place_libfranka::JointTrajectoryGoalConstPtr &goal) {

  std::string robot_IP;
  if (!nh_.getParam("robot_ip", robot_IP)) {
    ROS_ERROR_STREAM("Specificare l'indirizzo IP del robot.");
    as_.setAborted();
  }

  try {

    // Connessione al robot
    franka::Robot robot(robot_IP);

    // Set collision behavior
    // (https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html#a168e1214ac36d74ac64f894332b84534)
    setDefaultBehavior(robot);

    // Modello cinematico e dinamico del robot
    franka::Model model = robot.loadModel();

    // Lettura dello stato attuale del robot
    franka::RobotState initial_state;
    std::array<double, 7> q_init_{};

    // Doc:
    // https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html#a5b5ba0a4f2bfd20be963b05622e629e1

    bool success = false;
    double t = 0;
    double tf; // = goal->desired_tf; // s

    robot.control([this](const franka::RobotState &,
                         franka::Duration period) -> franka::JointPositions {
      
    });

    if (success) {
      result_.success = true;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }

  }

  catch (const franka::Exception &ex) {
    // print exception
    std::cout << ex.what() << "inside joint_traj_class" << std::endl;
  }
}

#include "pick_and_place_libfranka/GripperAction.h"

GripperAction::GripperAction(std::shared_ptr<franka::Robot> panda,
                             std::shared_ptr<std::mutex> robot_mutex)
    : gripper_move_as_(nh_, "gripper_move",
                       boost::bind(&GripperAction::GripperMoveCB, this, _1),
                       false) {

  if (!nh_.getParam("debugging", debugging)) {
    ROS_ERROR_STREAM("Param debugging not found.");
  }
  std::string robot_IP;
  if (!nh_.getParam("robot_ip", robot_IP)) {
    ROS_ERROR_STREAM("Param robot_ip not found.");
  }

  try {

    // Connect to robot
    robot_ = panda;

    // Connect to the gripper
    gripper_ = std::make_unique<franka::Gripper>(robot_IP);

    // Robot mutex
    robot_mutex_ = robot_mutex;

    // Start action servers
    gripper_move_as_.start();

    std::cout << "Creato oggetto GripperAction \n";

  } catch (const franka::Exception &ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
}

GripperAction::~GripperAction(void) {}

void GripperAction::GripperMoveCB(
    const pick_and_place_libfranka::GripperMoveGoalConstPtr &goal) {

  pick_and_place_libfranka::GripperMoveResult gripper_move_result;
  double speed = goal->desired_speed;
  double width = goal->desired_width;
  bool success = false;
  try {
    success = gripper_->move(width, speed);

  } catch (const franka::Exception &ex) {

    // print exception
    std::cout << ex.what() << std::endl;
  }
  gripper_move_result.success = success;
  gripper_move_as_.setSucceeded(gripper_move_result);
}

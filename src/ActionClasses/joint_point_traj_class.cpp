

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
#include <pick_and_place_libfranka/JointPointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>



  JointPointTrajAction::JointPointTrajAction(std::string name) :
    as_(nh_, name, boost::bind(&JointPointTrajAction::executeCB, this, _1), false),
    action_name_(name)    
  {
  
    as_.start();
  }

  JointPointTrajAction::~JointPointTrajAction(void)
  {
  }

  void JointPointTrajAction::executeCB(
      const pick_and_place_libfranka::JointPointTrajectoryGoalConstPtr &goal) {

    
    std::string robot_IP;
    if (!nh_.getParam("robot_ip", robot_IP)) {
      ROS_ERROR_STREAM("Specificare l'indirizzo IP del robot.");
      // as_.setAborted();
    }

   
    try {

      franka::Robot robot_(robot_IP);

      // Set collision behavior
      // (https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html#a168e1214ac36d74ac64f894332b84534)
      setDefaultBehavior(robot_);

      // Modello cinematico e dinamico del robot
      franka::Model model = robot_.loadModel();

      // Lettura dello stato attuale del robot
      franka::RobotState initial_state;
      std::array<double, 7> q_init_{};

      do {
        initial_state = robot_.readOnce();
        q_init_ = initial_state.q;
      } while (q_init_[3] == 0.0); // La quarta variabile di giunto è sempre negativa

      // Doc:
      // https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html#a5b5ba0a4f2bfd20be963b05622e629e1

      feedback_.time_left = goal->desired_tf;
      std::array<double, 7> q_command;
      std::array<double, 7> q_goal;
      for(int i = 0; i < 7; i++)
         q_goal[i] = goal->desired_conf[i];
      
      bool success = false;
      double t = 0;
      double tf = goal->desired_tf; // s
      
      robot_.control([&t, tf, q_init_, q_goal, goal, &success,
           &q_command,this](const franka::RobotState &,
                       franka::Duration period) -> franka::JointPositions {

            
            t += period.toSec();
            double tau = t / tf;
            for (int i = 0; i < 7; i++)
              q_command[i] =
                  q_init_[i] +
                  (q_goal[i] - q_init_[i]) *
                      (6 * pow(tau, 5) - 15 * pow(tau, 4) + 10 * pow(tau, 3));


            if (as_.isPreemptRequested() || !ros::ok()) {
              ROS_INFO("%s: Preempted", action_name_.c_str());
              // set the action state to preempted
              as_.setPreempted();
              return franka::MotionFinished(franka::JointPositions(q_command));
            }

            this->feedback_.time_left = goal->desired_tf - t;
            as_.publishFeedback(feedback_);

            if (t < tf) {
              return franka::JointPositions(q_command);
            }
            else {
              std::cout << "Motion finished \n";
              success = true;
              return franka::MotionFinished(franka::JointPositions(q_command));
            }

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
      std::cout << ex.what() << "inside joint_traj_point_class"<< std::endl;
    }

  
}



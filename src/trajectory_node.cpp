
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

  double t = 0.0;   // s
  double Ts = 10.0; // s
  double Tf = 10.0; // s

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

  {

    // Servizio per settare la configurazione iniziale nel trajectory planner.

    // TooN::Matrix<4, 4, double> n_T_e(TooN::Data(0.7071, 0.7071, 0.0, 0.0,
    // -0.7071,
    //                                             0.7071, 0.0, 0.0, 0.0,
    //                                             0.0, 1.0, 0.1034, 0.0, 0.0,
    //                                             0.0, 1.0));

    // sun::Panda panda(n_T_e, 1.0, "panda");
    // TooN::Matrix<4, 4, double> initial_pose = panda.fkine(initial_conf);

    // TooN::Vector<3> pi = TooN::makeVector(
    //     initial_pose[0][3], initial_pose[1][3],
    //     initial_pose[2][3]); // initial_transform.get_translation();
    //                          // // initial_position

    // sun::UnitQuaternion init_quat(initial_pose); // initial orientation

    // TooN::Matrix<3, 3> Rot_des = TooN::Data(1, 0, 0, 0, 0, 1, 0, -1, 0);
    // sun::UnitQuaternion final_quat(Rot_des);
    // sun::UnitQuaternion delta_quat =
    //     final_quat * inv(init_quat); // errore in terna base
    // sun::AngVec angvec = delta_quat.toangvec();

    // // Nota: la libreria vuole delta_quat definita in terna base.

    // // Generazione della traiettoria su primitiva di percorso di tipo
    // segmento:

    // std::cout << "Generazione della traiettoria in cartesiano \n";

    // TooN::Vector<3, double> pf({0.5, 0.5, 0.5});
    // // TooN::Vector<3, double> pf({0.655105430989015, 0.1096259365986445,
    // // 0.06857646438044779-0.01});

    // sun::Quintic_Poly_Traj qp_position(
    //     Tf, 0.0, 1.0); // polinomio quintico utilizzato per line_traj
    // sun::Quintic_Poly_Traj qp_orientation(
    //     Tf, 0.0, angvec.getAng()); // polinomio quintico utilizzato per
    //     quat_traj

    // sun::Line_Segment_Traj line_traj(pi, pf, qp_position);
    // sun::Rotation_Const_Axis_Traj quat_traj(init_quat, angvec.getVec(),
    //                                         qp_orientation);

    // // Nota: la traiettoria in orientamento è quella definita dal Delta_quat.
    // // Perchè vogliamo andare da init_quat (orientamento iniziale) a
    // final_quat
    // // (orientamento finale). Il metodo getquaternion(t) restituisce
    // DeltaQuat(t)
    // // * init_quat.

    // sun::Cartesian_Independent_Traj cartesian_traj(line_traj, quat_traj);

    // // Parametri CLIK
    // double Ts = 0.001;  // periodo s
    // double fs = 1 / Ts; // frequenza Hz

    // ros::Rate loop_rate(fs); // Hz

    // double gain = 0.5 * fs;
    // TooN::Vector<> qdot = TooN::Zeros(7); // velocità di giunto ritorno
    // TooN::Vector<6, int> mask =
    //     TooN::Ones;                   // maschera, se l'i-esimo elemento è
    //     zero allora l'i-esima
    //                                   // componente cartesiana non verrà
    //                                   usata per il calcolo
    //                                   // dell'errore
    // TooN::Vector<3> xd = TooN::Zeros; // velocità in translazione desiderata
    // TooN::Vector<3> w = TooN::Zeros;  // velocità angolare desiderata
    // TooN::Vector<6> error =
    //     TooN::Ones; // questo va "resettato" ogni volta prima del clik
    // TooN::Vector<7> qDH_k = initial_conf_;
    // sun::UnitQuaternion oldQ = init_quat;

    // sun::UnitQuaternion unit_quat_d = init_quat;
    // TooN::Vector<3, double> posizione_d = pi;

    // double begin = ros::Time::now().toSec();
    // double t;

    // std::cout << "Inizia moto " << std::endl;

    // while (ros::ok() &&
    //        !cartesian_traj.isCompleate(t)) // &&
    //        !cartesian_traj.isCompleate(t)
    // {

    //   t = ros::Time::now().toSec() - begin; // tempo trascorso
    //   posizione_d = cartesian_traj.getPosition(t);
    //   unit_quat_d = cartesian_traj.getQuaternion(0);
    //   xd = cartesian_traj.getLinearVelocity(t);
    //   w = cartesian_traj.getAngularVelocity(0);

    //   qDH_k = panda.clik(
    //       qDH_k,       //<- qDH attuale
    //       posizione_d, // <- posizione desiderata
    //       unit_quat_d, // <- quaternione desiderato
    //       oldQ,        // <- quaternione al passo precedente (per garantire
    //       la
    //                    // continuità)
    //       xd,          // <- velocità in translazione desiderata
    //       w,           //<- velocità angolare desiderata
    //       mask,        // <- maschera, se l'i-esimo elemento è zero allora
    //       l'i-esima
    //                    // componente cartesiana non verrà usata per il
    //                    calcolo
    //                    // dell'errore
    //       gain,        // <- guadagno del clik
    //       Ts,          // <- Ts, tempo di campionamento
    //       0.0,         // <- quadagno obj secondario
    //       TooN::Zeros(panda.getNumJoints()),

    //       // Return Vars
    //       qdot,  // <- variabile di ritorno velocità di giunto
    //       error, //<- variabile di ritorno errore
    //       oldQ   // <- variabile di ritorno: Quaternione attuale (N.B. qui
    //       uso oldQ
    //              // in modo da aggiornare direttamente la variabile oldQ e
    //              averla
    //              // già pronta per la prossima iterazione)
    //   );

    //   bool limits_exceeded =
    //       panda.exceededHardJointLimits(panda.joints_DH2Robot(qDH_k));
    //   pick_and_place_libfranka::TrajectoryPointStamped command_msg;
    //   trajectory_msgs::MultiDOFJointTrajectoryPoint traj_msg;

    //   // Pubblicazione comando in spazio giunti
    //   if (!limits_exceeded)
    //   {
    //     command_msg.header.stamp = ros::Time::now();
    //     for (int i = 0; i < 7; i++)
    //     {
    //       command_msg.point.positions.push_back(qDH_k[i]);
    //       command_msg.point.velocities.push_back(qdot[i]);
    //     }
    //     command_msg.finished = false;
    //   }
    //   else
    //   {
    //     throw std::runtime_error("Limiti di giunto superati");
    //   }

    //   command_pb.publish(command_msg);

    //   // // Comando in orientamento
    //   // traj_msg.transforms[0].rotation.x = unit_quat_d.getS();
    //   // TooN::Vector<3, double> vec_quat = unit_quat_d.getV();
    //   // traj_msg.transforms[0].rotation.y = vec_quat[0];
    //   // traj_msg.transforms[0].rotation.z = vec_quat[1];
    //   // traj_msg.transforms[0].rotation.w = vec_quat[2];

    //   // traj_pb.publish(traj_msg);

    //   // // Pubblicazione cinematica diretta ottenuta dalla q clik
    //   // TooN::Matrix<4,4,double> fkine = panda.fkine(qDH_k);
    //   // fkine_msg.transforms.resize(1);
    //   // fkine_msg.transforms[0].translation.x = fkine[0][3];
    //   // fkine_msg.transforms[0].translation.y = fkine[1][3];
    //   // fkine_msg.transforms[0].translation.z = fkine[2][3];

    //   // sun::UnitQuaternion quat_fkine(fkine);  // inizializzazione
    //   quaternione
    //   // con matrice 4x4 TooN::Vector<3, double> fkine_vec_quat =
    //   // quat_fkine.getV(); fkine_msg.transforms[0].rotation.x =
    //   // unit_quat_d.getS(); fkine_msg.transforms[0].rotation.y =
    //   // fkine_vec_quat[0]; fkine_msg.transforms[0].rotation.z =
    //   // fkine_vec_quat[1]; fkine_msg.transforms[0].rotation.w =
    //   // fkine_vec_quat[2];

    //   // fkine_pb.publish(fkine_msg);

    //   loop_rate.sleep();
    // }

    // // fine traiettoria, invio messaggio con finished = true
    // std::cout << "Moto terminato lato trajectory planner\n";
    // pick_and_place_libfranka::TrajectoryPointStamped final_msg;

    // for (int i = 0; i < 7; i++)
    // {
    //   final_msg.point.positions.push_back(qDH_k[i]);
    //   final_msg.point.velocities.push_back(qdot[i]);
    // }

    // final_msg.finished = true;

    // command_pb.publish(final_msg);
  }

  return 0;
}

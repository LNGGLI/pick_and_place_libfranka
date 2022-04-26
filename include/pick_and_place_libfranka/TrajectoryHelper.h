
#ifndef JOINT_TRAJ_HELPER

#define JOINT_TRAJ_HELPER
#pragma once

#include <cmath>
#include <mutex>
#include <thread>

// sun libraries
#include "pick_and_place_libfranka/Panda.h"
// Third party libraries
#include <TooN/TooN.h>
#include <ros/ros.h>

// msgs
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

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
  }

  return result;
}

/**
 * @brief Computes the coefficients of the polynomial functions used to
 * interpolate the given points. It uses 3rd degree polynomial functions and
 * the velocity at each path point is computed such that the robot doesn't
 * stop at each point. (Robotics Bruno Siciliano)
 * @param points: full trajectory as a
 * std::vector<trajectory_msgs::JointTrajectoryPoint>
 *
 * @return Vector of TooN Matrices: std::vector<TooN::Matrix<TooN::Dynamic, 4,
 * double>>. The vector will be composed of 7 matrices, one of each joint. Each
 * toon matrix contains the polynomial coefficents for the interpolation. Each
 * matrix is a N-1 x 4 where N is the number of given points (so N-1 polynomial
 * functions). The coefficients go from a0 to a3
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

/**
 * @brief computes a the inverse kinematics corresponding to the given
 * trajectory for the panda robot.
 * @param points vector of trajectory points that describes the desired task
 * space trajectory
 * @param initial_joint_state is used as a starting configuration for the clik.
 * This configuration, with time_from_start = 0, will be the first point of the
 * output.
 * @param n_T_e omogeneous transformation matrix from flange to EE.
 * @return the joint configuration corresponding to the desired cartesian in the
 * trajectory.
 * **/
std::vector<trajectory_msgs::JointTrajectoryPoint>
panda_clik(std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> &points,
           const std::array<double, 7> &initial_joint_state,
           const TooN::Matrix<4, 4, double> &n_T_e, const double &Ts) {

  // TooN::Matrix<4, 4, double> n_T_e(TooN::Data(0.7071, 0.7071, 0.0, 0.0,
  // -0.7071,
  //                                             0.7071, 0.0, 0.0, 0.0,
  //                                             0.0, 1.0, 0.1034, 0.0, 0.0,
  //                                             0.0, 1.0));

  // Create robot object

  sun::Panda panda(n_T_e, 1.0, "panda");

  // Result will be stored here
  std::vector<trajectory_msgs::JointTrajectoryPoint> joint_traj;

  if (points[0].time_from_start.toSec() == 0.0) {
    points.erase(points.begin());
  }

  // CLIK parameters
  double fs = 1 / Ts; // frequecy Hz

  double gain = 0.5 * fs;
  TooN::Vector<> qdot = TooN::Zeros(7); // velocità di giunto ritorno
  TooN::Vector<6, int> mask =
      TooN::Ones; // maschera, se l'i-esimo elemento è zero allora l'i-esima
                  // componente cartesiana non verrà usata per il calcolo
                  // dell'errore
  TooN::Vector<3> xd = TooN::Zeros; // velocità in translazione desiderata
  TooN::Vector<3> w = TooN::Zeros;  // velocità angolare desiderata
  TooN::Vector<6> error =
      TooN::Ones; // questo va "resettato" ogni volta prima del clik

  /** Initialize CLIK **/

  // qDH_k = current joint configuration
  TooN::Vector<7> qDH_k = TooN::makeVector(
      initial_joint_state[0], initial_joint_state[1], initial_joint_state[2],
      initial_joint_state[3], initial_joint_state[4], initial_joint_state[5],
      initial_joint_state[6]);

  // Old Q = current desired Q
  sun::UnitQuaternion oldQ = sun::UnitQuaternion(TooN::makeVector(
      points[0].transforms[0].rotation.w, points[0].transforms[0].rotation.x,
      points[0].transforms[0].rotation.y,
      points[0].transforms[0].rotation.z)); // used for continuity

  sun::UnitQuaternion unit_quat_d;

  // posizione_d = current desired position
  TooN::Vector<3, double> posizione_d;

  trajectory_msgs::JointTrajectoryPoint joint_point;

  // control variables
  int point = 0;
  double Tf = points.back().time_from_start.toSec();
  joint_traj.reserve((int)(Tf / Ts));
  double t = 0.0;

  while (t < Tf) {

    // Save joint_point
    joint_point.time_from_start =
        ros::Duration(t); // before t = t + Ts so that initial_conf
                          // will be the first joint configuration (t == 0)
    joint_point.positions.clear();
    for (int i = 0; i < 7; i++)
      joint_point.positions.push_back(qDH_k[i]);

    joint_traj.push_back(joint_point);

    // Compute next joint_point
    posizione_d = TooN::makeVector(points[point].transforms[0].translation.x,
                                   points[point].transforms[0].translation.y,
                                   points[point].transforms[0].translation.z);

    unit_quat_d = sun::UnitQuaternion(
        TooN::makeVector(points[point].transforms[0].rotation.w,
                         points[point].transforms[0].rotation.x,
                         points[point].transforms[0].rotation.y,
                         points[point].transforms[0].rotation.z));

    qDH_k = panda.clik(qDH_k,       //<- qDH attuale
                       posizione_d, // <- posizione desiderata
                       unit_quat_d, // <- quaternione desiderato
                       oldQ,        // <- quaternione al passo precedente (per
                                    // garantire la continuità)
                       xd,          // <- velocità in translazione desiderata
                       w,           //<- velocità angolare desiderata
                       mask, // <- maschera, se l'i-esimo elemento è zero
                             // allora l'i-esima componente cartesiana non
                             // verrà usata per il calcolo dell'errore
                       gain, // <- guadagno del clik
                       Ts,   // <- Ts, tempo di campionamento
                       0.0,  // <- quadagno obj secondario
                       TooN::Zeros(panda.getNumJoints()),

                       // Return Vars
                       qdot,  // <- variabile di ritorno velocità di giunto
                       error, //<- variabile di ritorno errore
                       oldQ   // <- variabile di ritorno: Quaternione attuale
                              // (N.B. qui uso oldQ  in modo da aggiornare
                              // direttamente la variabile oldQ
                       // e averla già pronta per la prossima iterazione)
    );

    // Update time. Do it after you have added the point so that for t = 0.0
    // you will have the initial configuration.
    t = t + Ts;

    // If t > time_from_start of next point you have to change the desired
    // point
    if (t >= points[point + 1].time_from_start.toSec()) {
      point++;
    }
  }

  std::cout << "Fine del clik \n";

  return joint_traj;
}

#endif
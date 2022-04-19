
#ifndef JOINT_TRAJ_HELPER

#define JOINT_TRAJ_HELPER
#pragma once

#include <cmath>
#include <mutex>
#include <thread>

// Third party libraries
#include <TooN/TooN.h>

// msgs
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

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

#endif
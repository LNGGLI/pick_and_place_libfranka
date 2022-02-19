// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// Librerie standard
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>

// Custom functions

#include "pick_and_place_libfranka/franka_node.h"


// Libreria Eigen Dense
#include <Eigen/Dense>

// Librerie Libfranka
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

// Contiene definizioni di funzioni utili, presente in libfranka
#include <pick_and_place_libfranka/examples_common.h>


// Utils
#include <ros/ros.h>
#include <pick_and_place_libfranka/check_realtime.h>
#include <TooN/TooN.h>

// Messaggi

#include <pick_and_place_libfranka/TrajectoryPointStamped.h>


/**
 * Programma di base C++ per il controllo in coppia del robot Panda. 
 *
 * Struttura del codice:
 * 1. Setup del robot
 * 2. Definizione dei parametri.
 * 3. Definizione del loop di controllo.
 * 4. Esecuzione del loop di controllo.
 * 
 * Nota: il programma non realizza nessun tipo di controllo ma può essere utilizzato come punto di partenza
 * per lo sviluppo di un algoritmo di controllo. Per avere un riscontro pratico si consiglia di
 * leggere il programma "Cartesian_Impedance_Control" nella cartella CustomPrograms così come
 * tutti gli esempi della cartella examples. 
 * Nota2: il Panda Robot compensa da se le coppie gravitazionali e gli attriti quindi non vanno considerati nel modello. 
 * 
 * Nota3: la documentazione della libreria Eigen3 è reperibile al link https://eigen.tuxfamily.org/dox/. 
 * Se si è familiari con l'utilizzo del Matlab si consiglia di dare uno sguardo al seguente cheat sheet:
 * https://eigen.tuxfamily.org/dox/AsciiQuickReference.txt 
 */

using namespace franka_control;

int main(int argc, char** argv) {


    
    ros::init(argc, argv, "frankalib_controller_node");
    ros::NodeHandle nh;

    std::string robot_IP;
    if (!nh.getParam("robot_ip", robot_IP))
    {
      ROS_ERROR_STREAM("Specificare l'indirizzo IP del robot.");
      return false;
    }

    ros::Publisher state_pb = nh.advertise<pick_and_place_libfranka::TrajectoryPointStamped>("/joint_state", 1);
    ros::Subscriber command_sub = nh.subscribe<pick_and_place_libfranka::TrajectoryPointStamped>("/joint_commands", 1 ,&franka_control::CommandCB);

    try {
      // Documentazione classe franka::Robot https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html
      // Documentazione classe franka::Model: https://frankaemika.github.io/libfranka/classfranka_1_1Model.html
      // Documentazione classe franka::RobotState: https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html

      // Connessione al robot 
        franka::Robot robot(robot_IP);
        
      // Set collision behavior (https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html#a168e1214ac36d74ac64f894332b84534)
        setDefaultBehavior(robot);

      // Modello cinematico e dinamico del robot
        franka::Model model = robot.loadModel();

      // Lettura dello stato attuale del robot e invio su topic /joint_state
        franka::RobotState initial_state = robot.readOnce();

        pick_and_place_libfranka::TrajectoryPointStamped state_msg; 
        state_msg.point.positions.resize(7);
        state_msg.header.stamp = ros::Time::now();

        std::array<double,7> q_init = initial_state.q;
        for(int i = 0; i < 7; i++){
          state_msg.point.positions[i] = q_init[i];
        }

        state_pb.publish(state_msg);



       double time = 0.0;

      
        auto generic_callback = [&](const franka::RobotState& robot_state,franka::Duration duration) -> franka::Torques {

       /** Inizio callback: durante il loop di controllo la callback viene eseguita con una frequenza
       * di 1Khz. La variabile robot_state viene implicitamente aggiornata ogni loop con la funzione robot.readOnce() **/
      
       // Update time: https://frankaemika.github.io/libfranka/classfranka_1_1Duration.html
        time += duration.toSec();
      
      
        Eigen::VectorXd  tau_des(7);

        // Calcolo 
        // coppie
        // di 
        // controllo

        std::array<double, 7> tau_des_array{};
        Eigen::VectorXd::Map(&tau_des_array[0], 7) = tau_des;


        return tau_des_array;
        };

      // Avvio del loop di controllo:
      // Doc: https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html#a5b5ba0a4f2bfd20be963b05622e629e1
        robot.control(generic_callback);

      } catch (const franka::Exception& ex) {
          // print exception
          std::cout << ex.what() << std::endl;
      }

      return 0;
}

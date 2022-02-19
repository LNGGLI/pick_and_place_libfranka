// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// Librerie standard
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>

// Libreria Eigen Dense
#include <Eigen/Dense>

// Librerie Libfranka
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

// Contiene definizioni di funzioni utili, presente in libfranka
#include <pick_and_place_libfranka/examples_common.h>


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


int main(int argc, char** argv) {

    if (argc != 2) {
        std::cerr << "Specificare l'indirizzo IP del robot." << std::endl;
        return -1;
    }

    try {
      // Documentazione classe franka::Robot https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html
      // Documentazione classe franka::Model: https://frankaemika.github.io/libfranka/classfranka_1_1Model.html
      // Documentazione classe franka::RobotState: https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html

      // Connessione al robot 
        franka::Robot robot(argv[1]);
        
      // Set collision behavior (https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html#a168e1214ac36d74ac64f894332b84534)
        setDefaultBehavior(robot);

      // Modello cinematico e dinamico del robot
        franka::Model model = robot.loadModel();

      // Lettura dello stato attuale del robot
        franka::RobotState initial_state = robot.readOnce();

      
      
      // A partire da una variabile robot_state si possono ottenere diverse informazioni, tra cui:
      // initial_state.O_T_EE è la matrice di trasformazione omogenea che descrive la posa 
      // dell'organo terminale rispetto alla terna zero del robot. 
      
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

      // Estrazione della posizione dalla matrice di trasformazione
        Eigen::Vector3d position_d(initial_transform.translation());

      // Estrazione dell'orientamento in forma di quaternione a partire dalla matrice di rotazione
        Eigen::Quaterniond orientation_d(initial_transform.linear());

      // Inizializzazione variabile time
        double time = 0.0;

      // Definizione della callback per il loop di controllo in coppia:
      // L'esempio definisce una lambda function ma nulla vieta di creare una funzione esterna. 
      
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

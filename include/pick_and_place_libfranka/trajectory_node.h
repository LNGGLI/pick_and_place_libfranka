
#pragma once

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

// Utils
#include <ros/ros.h>
#include <TooN/TooN.h>

// Messaggi e servizi

#include <pick_and_place_libfranka/TrajectoryPointStamped.h>



// namespace trajectory{


//     TooN::Vector<7,double> initial_conf_ = TooN::Zeros; 
    
//     bool initial_read = false;
//     double Tf = 15;

//     bool set_state(pick_and_place_libfranka::SetState::Request &req, pick_and_place_libfranka::SetState::Response &resp){

//         for(int i = 0 ; i < 7 ; i++){
//             initial_conf_[i] = req.initial_configuration[i];
//         }
        
//         if(initial_conf_[0]!= 0.0){

//             resp.success = true;
//             resp.message = "La condizione iniziale e' stata letta correttamente\n";
//             initial_read = true; // while(!initial_read) nel nodo trajectory

//             std::cout << "q ricevuta = " << initial_conf_ << "\n";
//             return true;
//         }
//         else{
            
//             resp.success = false;
//             resp.message = "La condizione iniziale non e' stata letta correttamente\n";
//             return false;
//         }   

        
//     }


   
// }
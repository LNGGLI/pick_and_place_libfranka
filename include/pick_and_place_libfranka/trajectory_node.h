
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
#include <pick_and_place_libfranka/SetState.h>



namespace trajectory{


    TooN::Vector<7,double> initial_conf = TooN::Zeros; 
    
    bool initial_read = false;
    double Tf = 15;

    bool set_state(pick_and_place_libfranka::SetState::Request &req, pick_and_place_libfranka::SetState::Response &resp){

        for(int i = 0 ; i < 7 ; i++){
            initial_conf[i] = req.initial_configuration[i];
        }
        
        if(initial_conf[0]!= 0.0){

            resp.success = true;
            resp.error = "La condizione iniziale è stata letta correttamente";
            initial_read = true; // while(!initial_read) nel nodo trajectory
            return true;
        }
        else{
            
            resp.success = false;
            resp.error = "La condizione iniziale non è stata letta correttamente";
            return false;
        }   

        
    }


   
}
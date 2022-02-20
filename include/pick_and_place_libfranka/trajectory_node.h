
#pragma once

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>


#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/SwitchController.h>
#include <ros/ros.h>

#include <franka_msgs/FrankaState.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>


using controller_manager_msgs::SwitchControllerRequest;
using controller_manager_msgs::SwitchControllerResponse;

namespace trajectory{


    TooN::Vector<7,double> initial_conf(7,0.0); 
    
    bool initial_read = false;
    double Tf = 15;

    void stateCB(const pick_and_place_libfranka::TrajectoryPointStamped::ConstPtr& msg){

        initial_conf = msg->point.positions;
        
        std::cout << "Configurazione iniziale q0:  \n";
        for(int i = 0; i < 7; i++)
            std::cout << initial_conf[i] << " ";

        std::cout << "\n";

        initial_read = true;
    }

    bool set_state(pick_and_place_libfranka::SetState::Request &req, pick_and_place_libfranka::SetState::Response &resp){

        for(int i = 0 ; i < 7 ; i++){
            initial_conf[i] = req.initial_configuration[i];
        }
        
        if(initial_conf[0]!= 0.0){

            resp.success = true;
            resp.error = "La condizione iniziale è stata letta correttamente";
            initial_read = true; // while(!initial_read) nel nodo trajectory
        }
        else{
            
            resp.success = false;
            resp.error = "La condizione iniziale non è stata letta correttamente";
        }   

        return true;
    }


   
}
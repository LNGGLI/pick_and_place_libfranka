
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
#include <sensor_msgs/JointState.h>

namespace trajectory
{
    bool state_read = false;
    std::array<double, 7> current_state;

    void stateCallBack(const sensor_msgs::JointState::ConstPtr &msg)
    {
        for (int i = 0; i < 7; i++)
            current_state[i] = msg->position[i];
        state_read = true;
    }

}
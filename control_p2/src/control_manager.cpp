#include "control_p2/control_manager.hpp"

ControlManager::ControlManager(){
    this->algorithm = Pursuit_Algorithm();
}

lart_msgs::msg::DynamicsCMD ControlManager::getDynamicsCMD(){

    lart_msgs::msg::DynamicsCMD controlOutput;

    if(!this->ready && !this->missionSet){
        // If not ready or mission not set, return zero commands
        controlOutput.rpm = 0;
        controlOutput.steering_angle = 0.0f;
        return controlOutput;
    }

    controlOutput = algorithm.calculate_control(this->currentPath, 
        this->currentPose, this->currentSpeed, this->currentSteering, this->missionSpeed);
    return controlOutput;
    
}

void ControlManager::set_ready(){
    this->ready = true;
}

void ControlManager::set_path(lart_msgs::msg::PathSpline path){
    this->currentPath = path;
}

void ControlManager::set_dynamics(lart_msgs::msg::Dynamics dynamics){
    this->currentSpeed = dynamics.rpm;
    this->currentSteering = dynamics.steering_angle;
}

void ControlManager::set_pose(geometry_msgs::msg::PoseStamped pose){
    this->currentPose = pose;
}

void ControlManager::set_mission(lart_msgs::msg::Mission mission){
    this->missionSet = true;

    switch(mission.data){
        case lart_msgs::msg::Mission::SKIDPAD:
        case lart_msgs::msg::Mission::AUTOCROSS:
        case lart_msgs::msg::Mission::TRACKDRIVE:
            this->missionSpeed = DEFAULT_MAX_SPEED;
            break;
        case lart_msgs::msg::Mission::ACCELERATION:
            this->missionSpeed = ACC_SPEED;
            break;
        case lart_msgs::msg::Mission::EBS_TEST:
            this->missionSpeed = EBS_SPEED;
            break;
        default:
            break;
    }
}



// Add a lot of diferent options that can be activated with flags
// Mostly an option to have a difrent aproach to the speed control
// One in wich the speed control is expected from the path planner
// And another where the speed control is done by the control node
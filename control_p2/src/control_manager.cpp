#include "control_p2/control_manager.hpp"

ControlManager::ControlManager(float mission_speed){
    this->algorithm = Pursuit_Algorithm(mission_speed);
}

lart_msgs::msg::DynamicsCMD ControlManager::getDynamicsCMD(){

    if(this->ready && this->missionSet){

        lart_msgs::msg::DynamicsCMD controlOutput = algorithm.calculate_control(this->currentPath, 
            this->currentPose, this->currentSpeed, this->currentSteering, this->missionSpeed);

        return controlOutput;

    }
}

void ControlManager::set_ready(){
    this->ready = true;
}

void ControlManager::set_maxSpeed(float speed){
    this->missionSpeed = speed;
    this->missionSet = true;
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



// Add a lot of diferent options that can be activated with flags
// Mostly an option to have a difrent aproach to the speed control
// One in wich the speed control is expected from the path planner
// And another where the speed control is done by the control node
#include "control_p2/target.hpp"

Target::Target(float mission_speed){
    this->algorithm = Pursuit_Algorithm();
    this->mission_speed = mission_speed;
}

lart_msgs::msg::DynamicsCMD Target::getDynamicsCMD(){
    // CALLS MULTIPLE FUNCTIONS ACORDING TO THE NECESSITIES OF THE FLAGS
    
    if(this->ready && this->mission_set){

        float steeringAngle = calculate_steeringAngle(this->path);

        float speed = calculate_speed(/* PARAMETERS */);

        //create dispatcher with speed and steering
        lart_msgs::msg::DynamicsCMD controlOutput = lart_msgs::msg::DynamicsCMD();
        controlOutput.rpm = speed;
        controlOutput.steering_angle = steeringAngle;

    }
}

void Target::set_ready(){
    this->ready = true;
}

void Target::set_maxSpeed(float speed){
    this->missionSpeed = speed;
    this->missionSet = true;
}

void Target::set_path(lart_msgs::msg::PathSpline path){
    this->currentPath = path;
}

void Target::set_dynamics(lart_msgs::msg::Dynamics dynamics){
    this->currentSpeed = dynamics.rpm;
    this->currentSteering = dynamics.steering_angle;
}

void Target::set_pose(geometry_msgs::msg::PoseStamped pose){
    this->currentPose = pose;
}

int Target::calculate_speed(lart_msgs::msg::PathSpline path, uint8_t currentSpeed, float missionSpeed){

    return 0;
}



// Add a lot of diferent options that can be activated with flags
// Mostly an option to have a difrent aproach to the speed control
// One in wich the speed control is expected from the path planner
// And another where the speed control is done by the control node
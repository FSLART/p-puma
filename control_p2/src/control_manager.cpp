#include "control_p2/control_manager.hpp"

ControlManager::ControlManager(){
}

lart_msgs::msg::DynamicsCMD ControlManager::getDynamicsCMD(){

    lart_msgs::msg::DynamicsCMD controlOutput;

    controlOutput = algorithm->calculate_control(this->currentPath, 
        this->currentPose, this->currentSpeed, this->currentSteering);
    
    // Add timestamp
    controlOutput.header.stamp = rclcpp::Clock().now();

    return controlOutput;
}

visualization_msgs::msg::Marker ControlManager::get_target_marker(){

    geometry_msgs::msg::PoseStamped target_point = algorithm->get_target_point();

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "base_footprint";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "pure_pursuit";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = target_point.pose.position.x;
    marker.pose.position.y = target_point.pose.position.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.lifetime = rclcpp::Duration::from_seconds(1);

    return marker;
}

void ControlManager::log_info(){
    //Write to console
    RCLCPP_INFO(rclcpp::get_logger("ControlManager"), "Current Speed: %.2f m/s | Current Steering: %.2f deg | Mission Speed: %.2f m/s", 
        this->currentSpeed, this->currentSteering, this->missionSpeed);

    //Obtain target point
    geometry_msgs::msg::PoseStamped target_point = this->algorithm->get_target_point();
    
    //Write to csv file
    std::ofstream log_file;
    log_file.open("control_log.csv", std::ios_base::app); // append mode
    log_file << this->currentSpeed << "," << this->currentSteering << "," << this->missionSpeed << "," << 
        this->currentPose.pose.position.x << "," << this->currentPose.pose.position.y << ","  << target_point.pose.position.x << "," << 
        target_point.pose.position.y << "," << rclcpp::Clock().now().seconds() << "\n";
    log_file.close();
}


void ControlManager::set_path(lart_msgs::msg::PathSpline path){
    this->currentPath = path;
}

void ControlManager::set_dynamics(lart_msgs::msg::Dynamics dynamics){
    this->currentSpeed = RPM_TO_MS(dynamics.rpm);
    this->currentSteering = dynamics.steering_angle;
}

void ControlManager::set_pose(geometry_msgs::msg::PoseStamped pose){
    this->currentPose = pose;
}

void ControlManager::set_missionSpeed(float missionSpeed){
    this->missionSpeed = missionSpeed;
    this->algorithm = new Pursuit_Algorithm(this->missionSpeed);
}

Pursuit_Algorithm * ControlManager::get_algorithm(){
    return this->algorithm;
}

lart_msgs::msg::PathSpline ControlManager::get_currentPath(){
    return this->currentPath;
}

geometry_msgs::msg::PoseStamped ControlManager::get_currentPose(){
    return this->currentPose;
}

float ControlManager::get_currentSpeed(){
    return this->currentSpeed;
}

float ControlManager::get_currentSteering(){
    return this->currentSteering;
}
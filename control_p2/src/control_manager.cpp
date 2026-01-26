#include "control_p2/control_manager.hpp"

ControlManager::ControlManager(){
    //Empty constructor
}

lart_msgs::msg::DynamicsCMD ControlManager::getDynamicsCMD(){

    lart_msgs::msg::DynamicsCMD controlOutput;

    controlOutput = algorithm->calculate_control(this->currentPath, 
        this->currentPose, this->currentSpeed, this->currentSteering);

    // compute max rpm in the same type as controlOutput.rpm and clamp
    auto max_rpm = static_cast<decltype(controlOutput.rpm)>(MS_TO_RPM(this->missionSpeed));
    controlOutput.rpm = std::clamp(controlOutput.rpm, static_cast<decltype(controlOutput.rpm)>(0), max_rpm);
    
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
    //New local path
    lart_msgs::msg::PathSpline local_path;
    
    //Eigen matrices for  transformation
    Eigen::Matrix3d T_robot_in_world = Eigen::Matrix3d::Identity();
    Eigen::Matrix3Xd worldMatrix(3, path.poses.size());

    //curremt car pose and heading
    geometry_msgs::msg::PoseStamped current_pose = this->get_currentPose();
    float theta = current_pose.pose.orientation.w;

    T_robot_in_world(0, 0) = cos(theta);
    T_robot_in_world(0, 1) = -sin(theta);
    T_robot_in_world(1, 0) = sin(theta);
    T_robot_in_world(1, 1) = cos(theta);
    T_robot_in_world(0, 2) = current_pose.pose.position.x;
    T_robot_in_world(1, 2) = current_pose.pose.position.y;


    for (size_t i = 0; i < path.poses.size(); ++i) {
        worldMatrix(0, i) = path.poses[i].pose.position.x;
        worldMatrix(1, i) = path.poses[i].pose.position.y;
        worldMatrix(2, i) = 1.0;
    }

    Eigen::Matrix3Xd localPoints = T_robot_in_world.inverse() * worldMatrix;

    std::vector<geometry_msgs::msg::PoseStamped> outputPoses;

    for (size_t i = 0; i < path.poses.size(); ++i) {
        geometry_msgs::msg::PoseStamped ps;
        
        ps.header.frame_id = "base_footprint";

        ps.pose.position.x = localPoints(0, i);
        ps.pose.position.y = localPoints(1, i);
        ps.pose.position.z = 0.0; // 2D Assumption

        ps.pose.orientation.x = 0.0;
        ps.pose.orientation.y = 0.0;
        ps.pose.orientation.z = 0.0;
        ps.pose.orientation.w = 1.0;

        outputPoses.push_back(ps);
    }

    local_path.header = path.header;
    local_path.poses = outputPoses;
    local_path.curvature = path.curvature; // Assuming curvature remains the same
    local_path.distance = path.distance; 
    
    this->currentPath = local_path;
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

// Add a lot of diferent options that can be activated with flags
// Mostly an option to have a difrent aproach to the speed control
// One in wich the speed control is expected from the path planner
// And another where the speed control is done by the control node
#ifndef TARGET_H_
#define TARGET_H_

#include "options.hpp"
#include ALGORITHM
#include <fstream>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class ControlManager {
    public: 
        ControlManager();
        void set_path(lart_msgs::msg::PathSpline path);
        void set_dynamics(lart_msgs::msg::Dynamics dynamics);
        void set_pose(geometry_msgs::msg::PoseStamped pose);
        void set_missionSpeed(float missionSpeed);
        lart_msgs::msg::DynamicsCMD getDynamicsCMD();
        Pursuit_Algorithm * get_algorithm();
        lart_msgs::msg::PathSpline get_currentPath();
        geometry_msgs::msg::PoseStamped get_currentPose();
        float get_currentSpeed();
        float get_currentSteering();
        visualization_msgs::msg::Marker get_target_marker();
        void log_info();
        vector<float> get_pid_debug();

    private:
        Pursuit_Algorithm *algorithm;
    protected:
        // Parameters
        float currentSpeed;
        float currentSteering;
        float missionSpeed;
        geometry_msgs::msg::PoseStamped currentPose;
        lart_msgs::msg::PathSpline currentPath;

    
};

#endif
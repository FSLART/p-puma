#ifndef ACC_PURSUIT_H_
#define ACC_PURSUIT_H_

#include "../utils.hpp"

#include <algorithm>
#include <optional>
#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;

class Pursuit_Algorithm {
    public:
        Pursuit_Algorithm(float missionSpeed);
        lart_msgs::msg::DynamicsCMD calculate_control(lart_msgs::msg::PathSpline path, geometry_msgs::msg::PoseStamped pose,
             float current_speed, float current_steering);

        geometry_msgs::msg::PoseStamped get_target_point();
    private:
        // functions
        float speed_to_lookahead(float speed);
        int fastRound(float x);
        float calculate_desiredSpeed(lart_msgs::msg::PathSpline path);
        float lowPassFilter(float input, float dt);
        
        // Parameters
        lart_msgs::msg::DynamicsCMD prevOutput;
        rclcpp::Time prevTime;

        int closest_point_index = -1;
        geometry_msgs::msg::PoseStamped target_point;
        float missionSpeed;
        VehicleModel vehicle = VehicleModel();

        // PID Controller
        PID_Controller pid_controller;
         
};

class PID_Controller{
    public:
        //Functions
        PID_Controller();
        float compute(float setpoint, float input);
            
    protected:
        float error, error_prev, error_sum;
};



#endif
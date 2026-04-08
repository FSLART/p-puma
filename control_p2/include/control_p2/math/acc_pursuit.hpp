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

class PID_Controller{
    public:
        //Functions
        PID_Controller() = default;
        PID_Controller(float kp, float ki, float kd);
        float compute(float setpoint, float input);
        void set_P(float kp);
        void set_I(float ki);
        void set_D(float kd);
            
    protected:
        float kp, ki, kd;
        float error, error_prev, error_sum;
};

class Pursuit_Algorithm {
    public:
        Pursuit_Algorithm(float missionSpeed, float lookahead_time, float tau, float kp, float ki, float kd);
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
        float lookahead_time;
        float tau;
        VehicleModel vehicle = VehicleModel();

        // PID Controller
        PID_Controller pid_controller;
         
};

#endif
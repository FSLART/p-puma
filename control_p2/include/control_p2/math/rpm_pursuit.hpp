#ifndef RPM_PURSUIT_H_
#define RPM_PURSUIT_H_

#include "../utils.hpp"
#include <rclcpp/rclcpp.hpp>

#include <optional>
#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
    
using namespace std;

class Pursuit_Algorithm {
    public:
        Pursuit_Algorithm(float missionSpeed, float lookahead_time, float tau, float kv, float curvature_gain, float kp, float ki, float kd) ;
        lart_msgs::msg::DynamicsCMD calculate_control(lart_msgs::msg::PathSpline path, geometry_msgs::msg::PoseStamped pose,
             float current_speed, float current_steering);

        geometry_msgs::msg::PoseStamped get_target_point();
        void set_missionSpeed(float missionSpeed);
        void set_lookahead_time(float lookahead_time);
        void set_tau(float tau);
        void set_kv(float kv);
        void set_curvature_gain(float curvature_gain);
        void set_kp(float kp);
        void set_ki(float ki);
        void set_kd(float kd);
    private:
        // functions
        float calculate_lookahead(float preview_curvature, float speed);
        int fastRound(float x);
        float calculate_desiredSpeed(float preview_curvature);
        float lowPassFilter(float input, float dt);
        float preview_abs_curvature(lart_msgs::msg::PathSpline path);
        
        // Parameters
        lart_msgs::msg::DynamicsCMD prevOutput;
        rclcpp::Time prevTime;

        int closest_point_index = -1;
        geometry_msgs::msg::PoseStamped target_point;
        float missionSpeed;
        float lookahead_time;
        float tau;
        float kv;
        float curvature_gain;
        VehicleModel vehicle = VehicleModel();
         
};

#endif
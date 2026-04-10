#ifndef LP_PURSUIT_H_
#define LP_PURSUIT_H_

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
        Pursuit_Algorithm(float missionSpeed, float lookahead_time, float tau, float kp, float ki, float kd);
        lart_msgs::msg::DynamicsCMD calculate_control(lart_msgs::msg::PathSpline path, geometry_msgs::msg::PoseStamped pose,
             float current_speed, float current_steering);

        geometry_msgs::msg::PoseStamped get_target_point();
        void set_missionSpeed(float missionSpeed);
        void set_lookahead_time(float lookahead_time);
        void set_tau(float tau);
        void set_kp(float kp);
        void set_ki(float ki);
        void set_kd(float kd);
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
         
};

#endif
#ifndef PURE_PURSUIT_H_
#define PURE_PURSUIT_H_

#include "../utils.hpp"

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
    private:
        // Auxiliary functions
        vector<array<float, 2>> transform_path(lart_msgs::msg::PathSpline path);
        float speed_to_lookahead(float speed);
        optional<array<float, 2>> get_closest_point(vector<array<float, 2>> path_points, float look_ahead_distance);
        int fastRound(float x);
        float calculate_desiredSpeed(lart_msgs::msg::PathSpline path);
        void keepAvgAngle(float new_angle);
        float getAvgAngle();
        void set_target_point(array<float, 2> closest_point);
        
        // Parameters
        int closest_point_index = -1;
        float grip_coefficient;
        float avg_angle[SIZE_AVG_ARRAY] = {0};
        int cycles = 0;
        array<float, 2> target_point;
        float missionSpeed;
        
};

#endif
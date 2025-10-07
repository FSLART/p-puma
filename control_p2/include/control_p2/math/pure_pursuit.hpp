#ifndef PURE_PURSUIT_H_
#define PURE_PURSUIT_H_
#include "../utils.hpp"

class Pursuit_Algorithm {
    public:
        Pursuit_Algorithm();
        lart_msgs::msg::DynamicsCMD calculate_control(lart_msgs::msg::PathSpline path, geometry_msgs::msg::PoseStamped pose,
             float current_speed, float current_steering, float mission_speed);
    private:
        // Auxiliary functions
        vector<array<float, 2>> transform_path(lart_msgs::msg::PathSpline path);
        float speed_to_lookahead(float speed);
        optional<array<float, 2>> get_closest_point(vector<array<float, 2>> path_points, float look_ahead_distance);
        int fastRound(float x);
        
        // Parameters
        int closest_point_index = -1;
        float grip_coefficient;
        float avg_angle[SIZE_AVG_ARRAY] = {0};
        int cycles = 0;
        
        float missionSpeed;
        
};

#endif
#include "control_p2/math/pure_pursuit.hpp"

Pursuit_Algorithm::Pursuit_Algorithm(float missionSpeed){
    this->missionSpeed = missionSpeed;
}

lart_msgs::msg::DynamicsCMD Pursuit_Algorithm::calculate_control(lart_msgs::msg::PathSpline path, geometry_msgs::msg::PoseStamped current_pose,
             float current_speed, float current_steering){

    //Declare variable to return
    lart_msgs::msg::DynamicsCMD control_output;

    // Transform the path to the car's coordinate system
    vector<array<float, 2>> path_points = transform_path(path);

    // Calculate look ahead point based on the speed with a min and max distances
    float look_ahead_distance = clamp(speed_to_lookahead(current_speed), MIN_LOOKAHEAD, MAX_LOOKAHEAD);

}

#include "control_p2/math/control_algorithm.hpp"
#include <algorithm>


Control_Algorithm::Control_Algorithm(float missionSpeed, float lookahead_time, float tau, float kv, float curvature_gain, float kp, float ki, float kd) {
    this->missionSpeed = missionSpeed;
    this->lookahead_time = lookahead_time;
    this->tau = tau;
    this->kv = kv;
    this->curvature_gain = curvature_gain;

    //Initialize previous output for the first iteration
    this->prevOutput.steering_angle = 0.0f;
    this->prevOutput.rpm = 0;
 
    //Intialize previous time
    this->prevTime = rclcpp::Clock().now();

    (void)kp; // Unused for this algorithm
    (void)ki; // Unused for this algorithm
    (void)kd; // Unused for this algorithm
 }

lart_msgs::msg::DynamicsCMD Control_Algorithm::calculate_control(lart_msgs::msg::PathArray path, geometry_msgs::msg::PoseStamped current_pose,
             float current_speed, float current_steering){

    //Ignore unused parameters
    (void)current_steering;

    //Declare variable to return
    lart_msgs::msg::DynamicsCMD control_output;

    //calculate look ahead distance 
    float abs_curvature = preview_abs_curvature(path);
    float look_ahead_distance = clamp(calculate_lookahead(abs_curvature, current_speed), MIN_LOOKAHEAD, MAX_LOOKAHEAD);

    // Define the target point
    this->closest_point_index = fastRound((look_ahead_distance)/SPACE_BETWEEN_POINTS);

    // 1. Create a tf2 Quaternion object
    tf2::Quaternion q;

    // 2. Convert the message quaternion to the tf2 object
    tf2::fromMsg(current_pose.pose.orientation, q);

    // 3. Get the yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // //trasform target to local
    float shiffet_x = path.points[this->closest_point_index].x - current_pose.pose.position.x;
    float shiffet_y = path.points[this->closest_point_index].y - current_pose.pose.position.y;
    float final_x = shiffet_x * cos(-yaw) - shiffet_y * sin(-yaw);
    float final_y = shiffet_x * sin(-yaw) + shiffet_y * cos(-yaw);

    //update target point
    this->target_point.pose.position.x = final_x;
    this->target_point.pose.position.y = final_y;

    //Get the euclidean distance to the target point
    float distance_to_target = std::sqrt(std::pow(this->target_point.pose.position.x, 2) + std::pow(this->target_point.pose.position.y, 2));

    // Get the dt since last call
    rclcpp::Time currentTime = rclcpp::Clock().now();
    float dt = (currentTime - this->prevTime).seconds();
    this->prevTime = currentTime;

    // Calculate desired speed and limit acceleration
    float desired_speed = calculate_desiredSpeed(abs_curvature);
    float desired_rpm = MS_TO_RPM(desired_speed);

    

    float prev_rpm = static_cast<float>(this->prevOutput.rpm);
    float speed_diff = desired_rpm - prev_rpm;
    if (speed_diff > MAX_RPM_DELTA) {
        desired_rpm = prev_rpm + MAX_RPM_DELTA;
    }

    //float desired_rpm = MS_TO_RPM(desired_speed);
    float desired_rpm_clamped = std::clamp(desired_rpm, 0.0f, (float)MS_TO_RPM(this->missionSpeed));

    float steering_angle = 0.0;

    if(abs(this->target_point.pose.position.y) >= 0.01){

        // Calculate angle between the closest point and (0,0) (because the point is returned relative to (0,0)) instead of the rear!!
        float alpha = atan2(this->target_point.pose.position.y, this->target_point.pose.position.x);

        // Calculate steering angle (pure pursuit algorithm)
        steering_angle = atan2(2 * WHEELBASE_M * sin(alpha), distance_to_target);
    }

    // Apply low pass filter to steering angle
    control_output.steering_angle = clamp(lowPassFilter(steering_angle, dt), (float)-MAX_WHEEL_ANGLE_RAD, (float)MAX_WHEEL_ANGLE_RAD);
    control_output.rpm = static_cast<decltype(control_output.rpm)>(desired_rpm_clamped);

    RCLCPP_INFO(rclcpp::get_logger("Control_Algorithm"), "steering_angle: %.2f, rpm: %d", control_output.steering_angle, control_output.rpm);

    //save previous output
    this->prevOutput = control_output;

    return control_output;
}

void Control_Algorithm::set_missionSpeed(float missionSpeed){
    this->missionSpeed = missionSpeed;
}

void Control_Algorithm::set_lookahead_time(float lookahead_time){
    this->lookahead_time = lookahead_time;
}

void Control_Algorithm::set_tau(float tau){
    this->tau = tau;
}

void Control_Algorithm::set_kv(float kv){
    this->kv = kv;
}

void Control_Algorithm::set_curvature_gain(float curvature_gain){
    this->curvature_gain = curvature_gain;
}

void Control_Algorithm::set_kp(float kp){
    // Not implemented for this algorithm   
}

void Control_Algorithm::set_ki(float ki){
    // Not implemented for this algorithm
}

void Control_Algorithm::set_kd(float kd){
    // Not implemented for this algorithm
}

int Control_Algorithm::fastRound(float x) {
    return static_cast<int>(x + 0.5f);
}

float Control_Algorithm::preview_abs_curvature(lart_msgs::msg::PathArray path){
    float sum_curvature = 0.0f;
    for(int i = 0; i < path.points.size(); i++){
        float curvature = std::abs(path.points[i].curvature);
        //float curvature = path.curvature[i];
        sum_curvature += curvature;
    }
    float preview_curvature = sum_curvature / path.points.size();
    return preview_curvature;
}

float Control_Algorithm::calculate_lookahead(float preview_curvature, float speed){
    // Lookahead distance increases with speed and decreases with curvature
    float look_ahead_distance = (MIN_LOOKAHEAD + this->lookahead_time * speed)/(1.0f + this->curvature_gain * preview_curvature); 
    return look_ahead_distance;
}

float Control_Algorithm::lowPassFilter(float input, float dt) {
        if (dt <= 0.0) return prevOutput.steering_angle;  // avoid division by zero
        float alpha = dt / (this->tau + dt);
        float output = alpha * input + (1.0 - alpha) * prevOutput.steering_angle;
        return output;
}

float Control_Algorithm::calculate_desiredSpeed(float preview_curvature){
    if(closest_point_index > -1){ 
        if(preview_curvature < 0.0001f){
            preview_curvature = 0.0001f; // Avoid division by zero
        }
        float velocity = std::sqrt(this->vehicle.get_grip_coefficient() * LART_GRAVITY * (1.0f/preview_curvature) * this->kv);
        velocity = clamp(velocity, 0.0f, this->missionSpeed);
        return velocity;
    }
    return 0.0f;
}

geometry_msgs::msg::PoseStamped Control_Algorithm::get_target_point()
{
    return this->target_point;
}
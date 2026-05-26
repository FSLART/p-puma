#include "control_p2/math/stanley_test.hpp"

Pursuit_Algorithm::Pursuit_Algorithm(float missionSpeed, float lookahead_time, float tau, float kv, float curvature_gain, float kp, float ki, float kd) {
    this->missionSpeed = missionSpeed;
    this->lookahead_time = lookahead_time;
    this->tau = tau;
    this->kv = kv;
    this->curvature_gain = curvature_gain;
    RCLCPP_INFO(rclcpp::get_logger("Pursuit_Algorithm"), " curvature_gain: %.2f", curvature_gain);
    //Initialize previous output for the first iteration
    this->prevOutput.steering_angle = 0.0f;
    this->prevOutput.acc_cmd = 0.0f;
    
    //Intialize previous time
    this->prevTime = rclcpp::Clock().now();

    this->pid_controller = PID_Controller(kp, ki, kd);
}

lart_msgs::msg::DynamicsCMD Pursuit_Algorithm::calculate_control(lart_msgs::msg::PathSpline path, geometry_msgs::msg::PoseStamped current_pose,
             float current_speed, float current_steering){

    //Ignore unused parameters
    (void)current_steering;

    lart_msgs::msg::DynamicsCMD control_output;
    
    //Get Curvature preview
    float abs_curvature = preview_abs_curvature(path);

    // Get the dt since last call
    rclcpp::Time currentTime = rclcpp::Clock().now();
    float dt = (currentTime - this->prevTime).seconds();
    this->prevTime = currentTime;

    // Calculate desired speed and limit acceleration
    float desired_speed = calculate_desiredSpeed(abs_curvature);

    RCLCPP_INFO(rclcpp::get_logger("Pursuit_Algorithm"), "Preview curvature: %.4f, Desired speed: %.2f", abs_curvature, desired_speed);

    //Limit change of desired speed per iteration
    float max_change = 1.0f; // Max change in speed per iteration
    float speed_diff = desired_speed - current_speed;
    if (speed_diff > max_change) {
        desired_speed = current_speed + max_change;
    }

    // Apply PID controller to define the aceleration
    // float acc_cmd = this->pid_controller.compute(desired_speed, current_speed);
    float acc_cmd = this->pid_controller.compute(desired_speed, current_speed, dt);

    acc_cmd = clamp(acc_cmd, MIN_SIG_VAL, MAX_SIG_VAL);

    //heading error and cross track error for future use
    float heading_error = calculate_heading_error(current_pose, path);

    float cross_track_error = calculate_cross_track_error(current_pose, path);

    //Calculate steering angle (stanley controller)

    float steering_angle = heading_error + atan((0.1 * cross_track_error) / (0.1 + current_speed));

    // Apply low pass filter to steering angle
    control_output.steering_angle = clamp(lowPassFilter(steering_angle, dt), (float)-MAX_WHEEL_ANGLE_RAD, (float)MAX_WHEEL_ANGLE_RAD);
    //control_output.steering_angle = clamp(steering_angle, (float)-MAX_WHEEL_ANGLE_RAD, (float)MAX_WHEEL_ANGLE_RAD);
    control_output.acc_cmd = acc_cmd;

    RCLCPP_INFO(rclcpp::get_logger("Pursuit_Algorithm"), "Heading error: %.2f, Cross track error: %.2f, final steering angle: %.2f , acc: %.2f", heading_error, cross_track_error, steering_angle, acc_cmd);

    //save previous output
    this->prevOutput = control_output;

    return control_output;
}

void Pursuit_Algorithm::set_missionSpeed(float missionSpeed){
    this->missionSpeed = missionSpeed;
}

void Pursuit_Algorithm::set_lookahead_time(float lookahead_time){
    this->lookahead_time = lookahead_time;
}

void Pursuit_Algorithm::set_tau(float tau){
    this->tau = tau;
}

void Pursuit_Algorithm::set_kv(float kv){
    this->kv = kv;
}  

void Pursuit_Algorithm::set_curvature_gain(float curvature_gain){
    this->curvature_gain = curvature_gain;
}

void Pursuit_Algorithm::set_kp(float kp){
    this->pid_controller.set_P(kp);
}

void Pursuit_Algorithm::set_ki(float ki){
    this->pid_controller.set_I(ki);
}

void Pursuit_Algorithm::set_kd(float kd){
    this->pid_controller.set_D(kd);
}


float Pursuit_Algorithm::calculate_heading_error(geometry_msgs::msg::PoseStamped current_pose, lart_msgs::msg::PathSpline path) {
    // Calculate the heading of the path at the closest point
    float path_yaw = atan2(path.poses[10].pose.position.y - path.poses[0].pose.position.y,
                            path.poses[10].pose.position.x - path.poses[0].pose.position.x);

    // Calculate the heading error
    float heading_error = path_yaw - atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w) * 2;

    // Normalize the heading error to the range [-pi, pi]
    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    while (heading_error < -M_PI) heading_error += 2 * M_PI;

    return heading_error;
}

float Pursuit_Algorithm::calculate_cross_track_error(geometry_msgs::msg::PoseStamped current_pose, lart_msgs::msg::PathSpline path) {
    // Calculate the cross track error as the distance from the current position to the closest point on the path
    float cross_track_error = sqrt(pow(current_pose.pose.position.x - path.poses[10].pose.position.x, 2) +
                                  pow(current_pose.pose.position.y - path.poses[10].pose.position.y, 2));
    return cross_track_error;
}

float Pursuit_Algorithm::lowPassFilter(float input, float dt) {
        if (dt <= 0.0) return prevOutput.steering_angle;  // avoid division by zero
        float alpha = dt / (this->tau + dt);
        float output = alpha * input + (1.0 - alpha) * prevOutput.steering_angle;
        return output;
}

float Pursuit_Algorithm::preview_abs_curvature(lart_msgs::msg::PathSpline path){
    float sum_curvature = 0.0f;
    for(int i = 0; i < PATH_SIZE; i++){
        float curvature = std::abs(path.curvature[i]);
        //float curvature = path.curvature[i];
        sum_curvature += curvature;
    }
    float preview_curvature = sum_curvature / PATH_SIZE;
    return preview_curvature;
}

float Pursuit_Algorithm::calculate_desiredSpeed(float preview_curvature){
    
    if(preview_curvature < 0.0001f){
        preview_curvature = 0.0001f; // Avoid division by zero
    }
    float velocity = std::sqrt(this->vehicle.get_grip_coefficient() * LART_GRAVITY * (1.0f/preview_curvature) * this->kv);
    //RCLCPP_INFO(rclcpp::get_logger("Pursuit_Algorithm"), "Velocity before: %.2f, kv: %.2f, radious: %.2f<", velocity, this->kv, (1.0f/preview_curvature));
    velocity = clamp(velocity, 0.0f, this->missionSpeed);
    return velocity;
}


geometry_msgs::msg::PoseStamped Pursuit_Algorithm::get_target_point()
{
    return this->target_point;
}

PID_Controller::PID_Controller(float kp, float ki, float kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    error = 0;
    error_prev = 0;
    error_sum = 0;
}

// float PID_Controller::compute(float setpoint, float input)
// {
//     error = setpoint - input;
//     error_sum += error;
//     error_prev = error;

//     float output = this->kp * error + this->ki * error_sum + this->kd * (error - error_prev);

//     if(output > MAX_SIG_VAL){
//         error_sum -= error;
//         output = MAX_SIG_VAL;
//     }else if(output < MIN_SIG_VAL){
//         error_sum -= error;
//         output = MIN_SIG_VAL;
//     }

//     return output;
// }

float PID_Controller::compute(float setpoint, float input, float dt)
{
    error = setpoint - input;

    // Integral
    error_sum += error * dt;

    // Derivative
    float derivative = (error - error_prev) / dt;

    // PID output
    float output =
        kp * error +
        ki * error_sum +
        kd * derivative;

    // Save previous error
    error_prev = error;

    // Clamp + anti-windup
    if(output > MAX_SIG_VAL){
        error_sum -= error * dt;
        output = MAX_SIG_VAL;
    }
    else if(output < MIN_SIG_VAL){
        error_sum -= error * dt;
        output = MIN_SIG_VAL;
    }

    return output;
}

void PID_Controller::set_P(float kp){
    this->kp = kp;
}
void PID_Controller::set_I(float ki){
    this->ki = ki;
}
void PID_Controller::set_D(float kd){
    this->kd = kd;
}

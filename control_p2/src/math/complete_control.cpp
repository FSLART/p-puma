#include "control_p2/math/control_algorithm.hpp"

// Longitudinal feedforward normalization: ff_cmd = a_ff / (mu_long * g).
// Must match whatever mu_drive/mu_brake the upstream velocity profile was
// planned with, otherwise the feedforward is mis-scaled.
static constexpr float MU_DRIVE = 0.5f;
static constexpr float MU_BRAKE = 0.5f;

Control_Algorithm::Control_Algorithm(float missionSpeed, float lookahead_time, float tau, float kv, float curvature_gain, float kp, float ki, float kd) {
    this->missionSpeed = missionSpeed;
    this->lookahead_time = lookahead_time;
    this->tau = tau;
    this->kv = kv;
    this->curvature_gain = curvature_gain;
    
    //Initialize previous output for the first iteration
    this->prevOutput.steering_angle = 0.0f;
    this->prevOutput.acc_cmd = 0.0f;
    
    //Intialize previous time
    this->prevTime = rclcpp::Clock().now();

    this->pid_controller = PID_Controller(kp, ki, kd);
}

lart_msgs::msg::DynamicsCMD Control_Algorithm::calculate_control(lart_msgs::msg::PathArray path, geometry_msgs::msg::PoseStamped current_pose,
             float current_speed, float current_steering){

    //Ignore unused parameters
    (void)current_steering;
    
    //Declare variable to return
    lart_msgs::msg::DynamicsCMD control_output;
    
    //calculate look ahead distance 
    float look_ahead_distance = clamp(calculate_lookahead(0.0,current_speed), MIN_LOOKAHEAD, MAX_LOOKAHEAD);
    
    // Define the target point
    this->closest_point_index = fastRound((look_ahead_distance)/SPACE_BETWEEN_POINTS);

    // 1. Create a tf2 Quaternion object
    tf2::Quaternion q;

    // 2. Convert the message quaternion to the tf2 object
    tf2::fromMsg(current_pose.pose.orientation, q);

    // 3. Get the yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    //trasform target to local
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
    if (dt <= 0.0f) {
        dt = 1.0f / FREQUENCY;                            // guard first tick / clock glitches
    }

    // Define if the path has velocity profile
    bool has_velocity_profile = false;
    if(path.points[0].velocity > -1.0f){
        has_velocity_profile = true;
    }

    float desired_speed;
    float ff_cmd = 0.0f;

    if(has_velocity_profile){
        // Trust the offline-planned profile instead of re-deriving a speed from live curvature.
        desired_speed = clamp(path.points[0].velocity, 0.0f, this->missionSpeed);

        // Feedforward acceleration (energy form) from the planned profile
        float a_ff = calculate_feedforward_accel(path);
        float mu_long = (a_ff >= 0.0f) ? MU_DRIVE : MU_BRAKE;
        ff_cmd = a_ff / (mu_long * LART_GRAVITY);
    } else {
        // No profile yet (e.g. first lap): fall back to the live curvature estimate
        float abs_curvature = preview_abs_curvature(path);
        desired_speed = calculate_desiredSpeed(abs_curvature);
    }

    control_output.target_ms = desired_speed;

    //Limit change of desired speed per iteration. Only needed to tame the
    //noisy live curvature estimate above; the planned profile is already
    //smooth and the feedforward term covers the launch/braking ramp instead.
    if(!has_velocity_profile){
        float max_change = 1.0f; // Max change in speed per iteration
        float speed_diff = desired_speed - current_speed;
        if (speed_diff > max_change) {
            desired_speed = current_speed + max_change;
        }
    }

    // Apply PID controller (feedback) and add the feedforward term on top
    // (ff_cmd is 0.0 in the no-profile fallback, so this reduces to the
    // previous pure-feedback behaviour). ff_cmd is passed in so the
    // combined-signal anti-windup can see the full command, not just fb_cmd.
    float fb_cmd = this->pid_controller.compute(desired_speed, current_speed, dt, ff_cmd);
    float acc_cmd = clamp(ff_cmd + fb_cmd, MIN_SIG_VAL, MAX_SIG_VAL);

    float steering_angle = 0.0;

    if(abs(this->target_point.pose.position.y) >= 0.01){

        // Calculate angle between the closest point and (0,0) (because the point is returned relative to (0,0)) instead of the rear!!
        float alpha = atan2(this->target_point.pose.position.y, this->target_point.pose.position.x);

        // Calculate steering angle (pure pursuit algorithm)
        steering_angle = atan2(2 * WHEELBASE_M * sin(alpha), distance_to_target);
    }

    // Apply low pass filter to steering angle
    control_output.steering_angle = clamp(lowPassFilter(steering_angle, dt), (float)-MAX_WHEEL_ANGLE_RAD, (float)MAX_WHEEL_ANGLE_RAD);
    //control_output.steering_angle = clamp(steering_angle, (float)-MAX_WHEEL_ANGLE_RAD, (float)MAX_WHEEL_ANGLE_RAD);
    control_output.acc_cmd = acc_cmd;

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
    this->pid_controller.set_P(kp);
}

void Control_Algorithm::set_ki(float ki){
    this->pid_controller.set_I(ki);
}

void Control_Algorithm::set_kd(float kd){
    this->pid_controller.set_D(kd);
}

int Control_Algorithm::fastRound(float x) {
    return static_cast<int>(x + 0.5f);
}

float Control_Algorithm::calculate_lookahead(float preview_curvature, float speed){
    (void)preview_curvature;
    float look_ahead_distance = this->lookahead_time * speed;
    return look_ahead_distance;
}

float Control_Algorithm::lowPassFilter(float input, float dt) {
        if (dt <= 0.0) return prevOutput.steering_angle;  // avoid division by zero
        float alpha = dt / (this->tau + dt);
        float output = alpha * input + (1.0 - alpha) * prevOutput.steering_angle;
        return output;
}

float Control_Algorithm::preview_abs_curvature(lart_msgs::msg::PathArray path){
    float sum_curvature = 0.0f;
    for(size_t i = 0; i < path.points.size(); i++){

        float curvature = std::abs(path.points[i].curvature);
        //float curvature = path.curvature[i];
        sum_curvature += curvature;
    }
    float preview_curvature = sum_curvature / path.points.size();
    return preview_curvature;
}

float Control_Algorithm::calculate_feedforward_accel(const lart_msgs::msg::PathArray &path){
    size_t n = path.points.size();
    if(n < 2){
        return 0.0f;
    }

    float v_i = path.points[0].velocity;
    float v_j = path.points[1].velocity;
    float ds = path.points[1].distance - path.points[0].distance;

    if(std::abs(ds) < 1e-4f){
        return 0.0f;
    }

    // Energy form: (v_j^2 - v_i^2) / (2*ds), evaluated at the car's current
    // position (path.points[0]) instead of the steering lookahead point.
    return (v_j * v_j - v_i * v_i) / (2.0f * ds);
}

float Control_Algorithm::calculate_desiredSpeed(float preview_curvature){
    if(closest_point_index > -1){ 
        if(preview_curvature < 0.0001f){
            preview_curvature = 0.0001f; // Avoid division by zero
        }
        float velocity = std::sqrt(this->vehicle.get_grip_coefficient() * LART_GRAVITY * (1.0f/preview_curvature));
        velocity = clamp(velocity, 0.0f, this->missionSpeed);
        return velocity;
    }
    return 0.0f;
}


geometry_msgs::msg::PoseStamped Control_Algorithm::get_target_point()
{
    return this->target_point;
}

PID_Controller::PID_Controller(float kp, float ki, float kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    error_prev = 0;
    error_sum = 0;
}

float PID_Controller::compute(float setpoint, float input, float dt, float ff_cmd)
{
    float error = setpoint - input;

    // Predict the COMBINED command with the current (not-yet-updated) integrator.
    float fb_preview = kp * error + ki * error_sum + kd * (error - error_prev) / dt;
    float cmd_preview = ff_cmd + fb_preview;

    // Freeze integration if the combined command is saturating AND the error
    // pushes it further into saturation (same sign).
    bool saturating = (std::abs(cmd_preview) > (float)MAX_SIG_VAL) &&
                      ((error >= 0.0f) == (cmd_preview >= 0.0f));
    if (!saturating) {
        error_sum += error * dt;
    }

    float derivative = (error - error_prev) / dt;
    error_prev = error;

    // feedback command (combine + clamp happens in calculate_control)
    return kp * error + ki * error_sum + kd * derivative;
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
void PID_Controller::reset(){
    this->error_sum = 0.0f;
    this->error_prev = 0.0f;
}

#include "control_p2/math/pp_profile.hpp"

/*
 * Longitudinal feedforward normalization.
 *
 *   ff_cmd = a_ff / (mu_long * g)
 *
 * so (mu_long * g) MUST equal the real acceleration the plant produces at
 * acc_cmd = 1. With the lart_to_pacsim bridge mapping acc_cmd -> per-wheel
 * torque = acc_cmd * 4 Nm, PacSim yields ~4.58 m/s^2 at acc_cmd = 1
 * (= mu_long*g with mu_long ~ 0.47). KEEP THESE CONSISTENT with:
 *   (1) the bridge torque scale (lart_to_pacsim_bridge.cpp), and
 *   (2) the 3-pass profile's mu_drive/mu_brake (midpoint_path.py),
 * otherwise the feedforward is biased (systematic over-/under-shoot).
 */
static constexpr float MU_LONG_DRIVE = 0.47f;
static constexpr float MU_LONG_BRAKE = 0.47f;

/*
 * Feedforward acceleration is the energy-form gradient of the planned speed
 * over a short preview:  a_ff = (v[k]^2 - v[0]^2) / (2 * k * ds).
 * k = 1 is the pure pointwise form (New Design); a few points smooths planner
 * noise and gives mild anticipation. At SPACE_BETWEEN_POINTS = 0.10 m,
 * FF_PREVIEW_POINTS = 5 -> 0.5 m.
 */
static constexpr int FF_PREVIEW_POINTS = 5;

Pursuit_Algorithm::Pursuit_Algorithm(float missionSpeed, float lookahead_time, float tau, float kv, float curvature_gain, float kp, float ki, float kd) {
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

lart_msgs::msg::DynamicsCMD Pursuit_Algorithm::calculate_control(lart_msgs::msg::PathSpline path, geometry_msgs::msg::PoseStamped current_pose,
             float current_speed, float current_steering){

    //Pure pursuit ignores the measured steering angle
    (void)current_steering;

    lart_msgs::msg::DynamicsCMD control_output;

    int n_pts = static_cast<int>(path.poses.size());
    if (n_pts < 2) {
        // No usable path — hold steering, command no acceleration.
        control_output.steering_angle = this->prevOutput.steering_angle;
        control_output.acc_cmd = 0.0f;
        return control_output;
    }

    // ------------------------------------------------------------------ //
    //  LATERAL  (identical to uff_pursuit)                               //
    // ------------------------------------------------------------------ //
    float look_ahead_distance = clamp(calculate_lookahead(current_speed), MIN_LOOKAHEAD, MAX_LOOKAHEAD);

    // Discrete target point on the rolling local path (index 0 ~ car).
    this->closest_point_index = fastRound(look_ahead_distance / SPACE_BETWEEN_POINTS);
    if (this->closest_point_index > n_pts - 1) {
        this->closest_point_index = n_pts - 1;            // bounds guard (uff lacks this)
    }

    tf2::Quaternion q;
    tf2::fromMsg(current_pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // transform target into the body frame
    float shiffet_x = path.poses[this->closest_point_index].pose.position.x - current_pose.pose.position.x;
    float shiffet_y = path.poses[this->closest_point_index].pose.position.y - current_pose.pose.position.y;
    float final_x = shiffet_x * cos(-yaw) - shiffet_y * sin(-yaw);
    float final_y = shiffet_x * sin(-yaw) + shiffet_y * cos(-yaw);

    this->target_point.pose.position.x = final_x;
    this->target_point.pose.position.y = final_y;

    // Euclidean chord length to the target (pure-pursuit denominator)
    float distance_to_target = std::sqrt(final_x * final_x + final_y * final_y);

    // ------------------------------------------------------------------ //
    //  dt                                                                //
    // ------------------------------------------------------------------ //
    rclcpp::Time currentTime = rclcpp::Clock().now();
    float dt = (currentTime - this->prevTime).seconds();
    this->prevTime = currentTime;
    if (dt <= 0.0f) {
        dt = 1.0f / FREQUENCY;                            // guard first tick / clock glitches
    }

    // ------------------------------------------------------------------ //
    //  LONGITUDINAL  (planned profile + feedforward — the new part)      //
    // ------------------------------------------------------------------ //
    float v_target = 0.0f;
    float a_ff = 0.0f;
    if (!path.velocity.empty()) {
        // planned speed at the car's current point (rolling-window index 0)
        v_target = clamp(static_cast<float>(path.velocity[0]), 0.0f, this->missionSpeed);
        a_ff = feedforward_accel(path);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("Pursuit_Algorithm"),
                    "PathSpline.velocity is empty — is step C (velocity field) wired? Commanding 0.");
    }

    // feedforward: expected accel as a fraction of available longitudinal grip
    float a_lon_max = (a_ff >= 0.0f) ? (MU_LONG_DRIVE * LART_GRAVITY)
                                     : (MU_LONG_BRAKE * LART_GRAVITY);
    float ff_cmd = a_ff / a_lon_max;

    // feedback with combined-signal anti-windup
    float fb_cmd = this->pid_controller.compute(v_target, current_speed, dt, ff_cmd);

    float acc_cmd = clamp(ff_cmd + fb_cmd, (float)MIN_SIG_VAL, (float)MAX_SIG_VAL);

    // ------------------------------------------------------------------ //
    //  STEERING (pure pursuit, Euclidean chord — identical to uff)       //
    // ------------------------------------------------------------------ //
    float steering_angle = 0.0;
    if (abs(this->target_point.pose.position.y) >= 0.01) {
        float alpha = atan2(this->target_point.pose.position.y, this->target_point.pose.position.x);
        steering_angle = atan2(2 * WHEELBASE_M * sin(alpha), distance_to_target);
    }

    control_output.steering_angle = clamp(lowPassFilter(steering_angle, dt), (float)-MAX_WHEEL_ANGLE_RAD, (float)MAX_WHEEL_ANGLE_RAD);
    control_output.acc_cmd = acc_cmd;

    //save previous output
    this->prevOutput = control_output;

    return control_output;
}

// Energy-form feedforward acceleration at the car (window index 0).
float Pursuit_Algorithm::feedforward_accel(const lart_msgs::msg::PathSpline& path) {
    int n = static_cast<int>(path.velocity.size());
    if (n < 2) {
        return 0.0f;
    }
    int k = FF_PREVIEW_POINTS;
    if (k > n - 1) {
        k = n - 1;
    }
    float v0 = static_cast<float>(path.velocity[0]);
    float vk = static_cast<float>(path.velocity[k]);
    float dist = k * SPACE_BETWEEN_POINTS;
    return (vk * vk - v0 * v0) / (2.0f * dist);
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

int Pursuit_Algorithm::fastRound(float x) {
    return static_cast<int>(x + 0.5f);
}

float Pursuit_Algorithm::calculate_lookahead(float speed){
    // velocity-proportional lookahead (caller clamps to [MIN, MAX]) — same as uff
    return this->lookahead_time * speed;
}

float Pursuit_Algorithm::lowPassFilter(float input, float dt) {
    if (dt <= 0.0) return prevOutput.steering_angle;  // avoid division by zero
    float alpha = dt / (this->tau + dt);
    float output = alpha * input + (1.0 - alpha) * prevOutput.steering_angle;
    return output;
}

geometry_msgs::msg::PoseStamped Pursuit_Algorithm::get_target_point()
{
    return this->target_point;
}

// --------------------------------------------------------------------------- //
//  PID with combined-signal anti-windup                                       //
// --------------------------------------------------------------------------- //

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

#include "control_p2/math/lp_pursuit.hpp"

Pursuit_Algorithm::Pursuit_Algorithm(float missionSpeed) {
    this->missionSpeed = missionSpeed;
    
    //Initialize previous output for the first iteration
    this->prevOutput.steering_angle = 0.0f;
    this->prevOutput.acc_cmd = 0.0f;
    
    //Intialize previous time
    this->prevTime = rclcpp::Clock().now();

    this->pid_controller = PID_Controller();
}

lart_msgs::msg::DynamicsCMD Pursuit_Algorithm::calculate_control(lart_msgs::msg::PathSpline path, geometry_msgs::msg::PoseStamped current_pose,
             float current_speed, float current_steering){

    //Declare variable to return
    lart_msgs::msg::DynamicsCMD control_output;

    // Calculate look ahead point based on the speed with a min and max distances
    float look_ahead_distance = clamp(speed_to_lookahead(current_speed), MIN_LOOKAHEAD, MAX_LOOKAHEAD);

    // Define the target point
    this->closest_point_index = fastRound((look_ahead_distance)/SPACE_BETWEEN_POINTS);
    this->target_point = path.poses[this->closest_point_index];

    // Get the dt since last call
    rclcpp::Time currentTime = rclcpp::Clock().now();
    float dt = (currentTime - this->prevTime).seconds();
    this->prevTime = currentTime;

    // Calculate desired speed and limit acceleration
    float desired_speed = calculate_desiredSpeed(path);

    // Apply PID controller to define the aceleration
    float acc_cmd = this->pid_controller.compute(desired_speed, current_speed);
    acc_cmd = clamp(acc_cmd, MIN_SIG_VAL, MAX_SIG_VAL);

    // If the target point is in front of the car then consider the desired angle to be 0
    if(this->target_point.pose.position.y == 0){

        // Apply low pass filter to steering angle towards 0
        control_output.steering_angle = lowPassFilter(0.0f, dt);
        control_output.acc_cmd = acc_cmd;

        //save previous output
        this->prevOutput = control_output;

        return control_output;
    }
    

    // Calculate angle between the closest point and (0,0) (because the point is returned relative to (0,0)) instead of the rear!!
    float alpha = atan2(this->target_point.pose.position.y, this->target_point.pose.position.x - DEFAULT_IMU_TO_REAR_AXLE);

    // Calculate steering angle (pure pursuit algorithm)
    float steering_angle = atan2(2 * WHEELBASE_M * sin(alpha), look_ahead_distance);

    // Apply low pass filter to steering angle
    control_output.steering_angle = lowPassFilter(steering_angle, dt);
    control_output.acc_cmd = acc_cmd;

    //save previous output
    this->prevOutput = control_output;

    return control_output;
}

int Pursuit_Algorithm::fastRound(float x) {
    return static_cast<int>(x + 0.5f);
}

float Pursuit_Algorithm::speed_to_lookahead(float speed){
    float look_ahead_distance = MIN_LOOKAHEAD + LOOKAHEAD_TIME * speed;
    return look_ahead_distance;
}

float Pursuit_Algorithm::lowPassFilter(float input, float dt) {
        if (dt <= 0.0) return prevOutput.steering_angle;  // avoid division by zero
        float alpha = dt / (TAU + dt);
        float output = alpha * input + (1.0 - alpha) * prevOutput.steering_angle;
        return output;
}

float Pursuit_Algorithm::calculate_desiredSpeed(lart_msgs::msg::PathSpline path){
    if(closest_point_index > -1){
        float curvature = std::abs(path.curvature[closest_point_index]);

        if(curvature < 0.0001f){
            curvature = 0.0001f; // Avoid division by zero
        }

        float velocity = std::sqrt(this->vehicle.get_grip_coefficient() * LART_GRAVITY * (1.0f/curvature));
        velocity = clamp(velocity, 0.0f, this->missionSpeed);

        return velocity;
    }
    return 0.0f;
}

geometry_msgs::msg::PoseStamped Pursuit_Algorithm::get_target_point()
{
    return this->target_point;
}

PID_Controller::PID_Controller()
{
    error = 0;
    error_prev = 0;
    error_sum = 0;
}

float PID_Controller::compute(float setpoint, float input)
{
    error = setpoint - input;
    error_sum += error;
    error_prev = error;

    float output = KP * error + KI * error_sum + KD * (error - error_prev);

    if(output > MAX_SIG_VAL){
        error_sum -= error;
        output = MAX_SIG_VAL;
    }else if(output < MIN_SIG_VAL){
        error_sum -= error;
        output = MIN_SIG_VAL;
    }

    return output;
}

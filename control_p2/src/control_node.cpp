#include "control_p2/control_node.hpp"

using std::placeholders::_1;

ControlP2::ControlP2() : Node("control_node")
{
    /*------------------------------------------------------------------------------*/
    /*                                   ROS PARAMS                                 */
    /*------------------------------------------------------------------------------*/
    
    //Flags
    this->declare_parameter<bool>("sim_mode", false);
    this->declare_parameter<bool>("log_info", false);
    this->declare_parameter<bool>("target_marker_visible", true);
    this->declare_parameter<bool>("fsl_flag", false);
    this->declare_parameter<bool>("acc_mode", false);

    //Mission speeds
    this->declare_parameter<float>("default_max_speed", 2.0f);
    this->declare_parameter<float>("acc_speed", 2.0f);
    this->declare_parameter<float>("skidpad_speed", 2.0f);
    this->declare_parameter<float>("ebs_speed", 2.0f);
    this->declare_parameter<float>("fsl_speed", 3.0f);

    //Lookahead tuning
    this->declare_parameter<float>("lookahead_time", 0.5f);

    //Algorithm tuning
    this->declare_parameter<float>("tau", 0.01f);
    this->declare_parameter<float>("kv", 0.2f);
    this->declare_parameter<float>("curvature_gain", 2.0f);
    this->declare_parameter<float>("kp", 1.0f);
    this->declare_parameter<float>("ki", 0.1f);
    this->declare_parameter<float>("kd", 0.05f);

    this->get_parameter("sim_mode", sim_mode);
    this->get_parameter("log_info", log_info);
    this->get_parameter("target_marker_visible", target_marker_visible);
    this->get_parameter("fsl_flag", fsl_flag);
    this->get_parameter("acc_mode", acc_mode);
    
    this->get_parameter("default_max_speed",default_max_speed);
    this->get_parameter("acc_speed",acc_speed);
    this->get_parameter("skidpad_speed",skidpad_speed);
    this->get_parameter("ebs_speed",ebs_speed);
    this->get_parameter("fsl_speed",fsl_speed);
    this->get_parameter("lookahead_time", lookahead_time);
    this->get_parameter("tau", tau);
    this->get_parameter("kv", kv);
    this->get_parameter("curvature_gain", curvature_gain);
    this->get_parameter("kp", kp);
    this->get_parameter("ki", ki);
    this->get_parameter("kd", kd);


    RCLCPP_INFO(this->get_logger(), "Control node initialized with parameters: sim_mode: %d, log_info: %d, target_marker_visible: %d, fsl_flag: %d, acc_mode: %d, fsl_speed: %.2f, default_max_speed: %.2f, acc_speed: %.2f, skidpad_speed: %.2f, ebs_speed: %.2f, lookahead_time: %.2f, tau: %.2f, kv: %.2f, curvature_gain: %.2f, kp: %.2f, ki: %.2f, kd: %.2f",
        sim_mode, log_info, target_marker_visible, fsl_flag, acc_mode, fsl_speed, default_max_speed, acc_speed, skidpad_speed, ebs_speed, lookahead_time, tau, kv, curvature_gain, kp, ki, kd);

    /*------------------------------------------------------------------------------*/
    /*                                   PUBLISHERS                                 */
    /*------------------------------------------------------------------------------*/
    
    dynamics_torque_publisher = this->create_publisher<lart_msgs::msg::DynamicsCMD>(TOPIC_CONTROL_TORQUE_TARGET, 10);

    dynamics_rpm_publisher = this->create_publisher<lart_msgs::msg::DynamicsCMD>(TOPIC_CONTROL_RPM_TARGET, 10);

    marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>(TOPIC_TARGET_MARKER, 10);

    lookahead_publisher = this->create_publisher<std_msgs::msg::Float32>("lookahead_distance", 10);

    /*------------------------------------------------------------------------------*/
    /*                                PUBLISHERS TIMER                              */
    /*------------------------------------------------------------------------------*/

    // Turn Hz into duration
    std::chrono::duration<double> interval = std::chrono::duration<double>(1.0 / FREQUENCY);

    control_timer = this->create_wall_timer(interval,
        std::bind(&ControlP2::dispatchDynamicsCMD, this));


    /*------------------------------------------------------------------------------*/
    /*                                  SUBSCRIBERS                                 */
    /*------------------------------------------------------------------------------*/

    path_subscriber = this->create_subscription<lart_msgs::msg::PathSpline>(
        TOPIC_PATH, 10, std::bind(&ControlP2::path_callback, this, _1));

    dynamics_subscriber = this->create_subscription<lart_msgs::msg::Dynamics>(
        TOPIC_CONTROL_FEEDBACK, 10, std::bind(&ControlP2::dynamics_callback, this, _1));

    state_subscriber = this->create_subscription<lart_msgs::msg::State>(
        TOPIC_STATE_PC, 10, std::bind(&ControlP2::state_callback, this, _1));

    mission_subscriber = this->create_subscription<lart_msgs::msg::Mission>(
        TOPIC_MISSION_PC, 10, std::bind(&ControlP2::mission_callback, this, _1));

    position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        TOPIC_SLAM_POSE, 10, std::bind(&ControlP2::pose_callback, this, _1));
    
    lap_subscriber = this->create_subscription<lart_msgs::msg::SlamStats>(
        TOPIC_STATS, 10, std::bind(&ControlP2::lap_callback, this, _1));

    /*------------------------------------------------------------------------------*/
    /*                                UPDATE PARAMS                                 */    
    /*------------------------------------------------------------------------------*/
    
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ControlP2::parametersCallback, this, std::placeholders::_1));

    /*------------------------------------------------------------------------------*/
    /*                            CLASS INITIALIZATION                              */
    /*------------------------------------------------------------------------------*/
    control_manager = new ControlManager();
    
    /*------------------------------------------------------------------------------*/
    /*                        SIMULATION MODE INITIALIZATION                        */
    /*------------------------------------------------------------------------------*/
    if(sim_mode){
        RCLCPP_WARN(this->get_logger(), "SIMULATION MODE ACTIVE: Starting with mission speed");
        this->missionSet = true;
        this->current_mission = lart_msgs::msg::Mission::TRACKDRIVE; // Default mission for simulation
        this->ready = true;
        this->control_manager->initialize_algorithm(default_max_speed, lookahead_time, tau, kv, curvature_gain, kp, ki, kd);
    }

}

void ControlP2::state_callback(const lart_msgs::msg::State::SharedPtr msg)
{

    switch (msg->data)
    {
    case lart_msgs::msg::State::DRIVING:
        if(!this->ready){
            RCLCPP_INFO(this->get_logger(), "State callback received: %d", msg->data);
            this->checkTimeStamp();
        }
        break;

    case lart_msgs::msg::State::FINISH:
        this->race_finished = true;
        break;

    case lart_msgs::msg::State::EMERGENCY:
        this->cleanUp();
        break;
    
    default:
        break;
    }
}

void ControlP2::mission_callback(const lart_msgs::msg::Mission::SharedPtr msg)
{

    //RCLCPP_INFO(this->get_logger(), "Mission received: %d", msg->data);
    if(this->missionSet){
        return;
    }

    this->missionSet = true;

    float missionSpeed = 0.0;

    switch(msg->data){
        case lart_msgs::msg::Mission::SKIDPAD:
        case lart_msgs::msg::Mission::AUTOCROSS:
        case lart_msgs::msg::Mission::TRACKDRIVE:
            missionSpeed = default_max_speed;
            break;
        case lart_msgs::msg::Mission::ACCELERATION:
            missionSpeed = acc_speed;
            break;
        case lart_msgs::msg::Mission::EBS_TEST:
            missionSpeed = ebs_speed;
            break;
        default:
            break;
    }

    //Save current mission
    this->current_mission = msg->data;

    this->control_manager->initialize_algorithm(missionSpeed, lookahead_time, tau, kv, curvature_gain, kp, ki, kd);
}

void ControlP2::lap_callback(const lart_msgs::msg::SlamStats::SharedPtr msg)
{
    if(this->missionSet && this->current_mission == lart_msgs::msg::Mission::TRACKDRIVE && msg->lap_count == 1 && this->fsl_flag){
        RCLCPP_WARN(this->get_logger(), "First lap completed, activating FSL mode");
        this->fsl_flag = false;
        this->control_manager->set_missionSpeed(fsl_speed);
    }
}

void ControlP2::path_callback(const lart_msgs::msg::PathSpline::SharedPtr msg)
{
    // save current path
    this->control_manager->set_path(*msg);
}

void ControlP2::dynamics_callback(const lart_msgs::msg::Dynamics::SharedPtr msg)
{
    // save current speed
    this->control_manager->set_dynamics(*msg);
}

void ControlP2::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // save current position from slam
    //geometry_msgs::msg::PoseStamped pose;
    this->control_manager->set_pose(*msg); //*msg
}

void ControlP2::dispatchDynamicsCMD()
{
    // Check if we have received the first driving signal and if the mission has been set
    if(!this->ready || !this->missionSet ){
        if(!this->drivingSignalTimeStamp.has_value())
            RCLCPP_WARN(this->get_logger(), "Control node not ready or mission not set");
        return;
    }

    // Check if we have received a path
    if(this->control_manager->get_currentPath().poses.empty()){
        RCLCPP_WARN(this->get_logger(), "No path received yet");
        return;
    }

    lart_msgs::msg::DynamicsCMD control_output = this->control_manager->getDynamicsCMD();

    if(this->race_finished){
        control_output.rpm = 0;
        control_output.acc_cmd = 0.0;
    }

    //Send dynamics in rpm or acceleration
    if(this->acc_mode){
        dynamics_torque_publisher->publish(control_output);
    }else{
        dynamics_rpm_publisher->publish(control_output);
    }

    // // publish dynamics command
    // this->dynamics_publisher->publish(control_output);
    RCLCPP_INFO(this->get_logger(), "Published DynamicsCMD: rpm: %d, steering_angle: %f, acc_cmd: %f", control_output.rpm, control_output.steering_angle, control_output.acc_cmd);

    // publish target marker
    if(target_marker_visible){
        visualization_msgs::msg::Marker target_marker = this->control_manager->get_target_marker();
        this->marker_publisher->publish(target_marker);
    }

    // log info
    if(log_info){
        this->control_manager->log_info();
    }

    // std_msgs::msg::Float32 lookahead_msg;
    // lookahead_msg.data = this->control_manager->get_lookahead_distance();
    // this->lookahead_publisher->publish(lookahead_msg);
}

rcl_interfaces::msg::SetParametersResult ControlP2::parametersCallback(const std::vector<rclcpp::Parameter> &params)
{

    for (const auto &param : params)
    {
        const std::string &name = param.get_name();

        if (name == "log_info") {
            log_info = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "log_info set to: %d", log_info);
        } 
        else if (name == "target_marker_visible") {
            target_marker_visible = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "target_marker_visible set to: %d", target_marker_visible);
        } 
        else if (name == "default_max_speed") {
            default_max_speed = param.as_double();
            this->control_manager->set_missionSpeed(default_max_speed);
            RCLCPP_INFO(this->get_logger(), "default_max_speed set to: %f", default_max_speed);
        } 
        else if (name == "acc_speed") {
            acc_speed = param.as_double();
            this->control_manager->set_missionSpeed(acc_speed);
            RCLCPP_INFO(this->get_logger(), "acc_speed set to: %f", acc_speed);
        }
        else if (name == "skidpad_speed") {
            skidpad_speed = param.as_double();
            this->control_manager->set_missionSpeed(skidpad_speed);
            RCLCPP_INFO(this->get_logger(), "skidpad_speed set to: %f", skidpad_speed);
        }
        else if (name == "ebs_speed") {
            ebs_speed = param.as_double();
            this->control_manager->set_missionSpeed(ebs_speed);
            RCLCPP_INFO(this->get_logger(), "ebs_speed set to: %f", ebs_speed);
        } 
        else if (name == "lookahead_time") {
            lookahead_time = param.as_double();
            this->control_manager->set_lookahead_time(lookahead_time);
            RCLCPP_INFO(this->get_logger(), "lookahead_time set to: %f", lookahead_time);
        } 
        else if (name == "tau") {
            tau = param.as_double();
            this->control_manager->set_tau(tau);
            RCLCPP_INFO(this->get_logger(), "tau set to: %f", tau);
        } 
        else if (name == "kv") {
            kv = param.as_double();
            this->control_manager->set_kv(kv);
            RCLCPP_INFO(this->get_logger(), "kv set to: %f", kv);
        }
        else if (name == "curvature_gain") {
            curvature_gain = param.as_double();
            this->control_manager->set_curvature_gain(curvature_gain);
            RCLCPP_INFO(this->get_logger(), "curvature_gain set to: %f", curvature_gain);
        }
        else if (name == "kp") {
            kp = param.as_double();
            this->control_manager->set_kp(kp);
            RCLCPP_INFO(this->get_logger(), "kp set to: %f", kp);
        } 
        else if (name == "ki") {
            ki = param.as_double();
            this->control_manager->set_ki(ki);
            RCLCPP_INFO(this->get_logger(), "ki set to: %f", ki);
        } 
        else if (name == "kd") {
            kd = param.as_double();
            this->control_manager->set_kd(kd);
            RCLCPP_INFO(this->get_logger(), "kd set to: %f", kd);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Parameters updated at runtime");
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    return result;
}

void ControlP2::checkTimeStamp()
{
    if(!this->drivingSignalTimeStamp.has_value()){
        this->drivingSignalTimeStamp = std::chrono::steady_clock::now();
        RCLCPP_WARN(this->get_logger(), "First driving signal received");
        return;
    }

    auto now = std::chrono::steady_clock::now();
    auto diff = now - this->drivingSignalTimeStamp.value();
    double seconds = std::chrono::duration<double>(diff).count();

    RCLCPP_INFO(this->get_logger(), "seconds since driving signal: %f", seconds);

    if (seconds > 3.0)
    {
        RCLCPP_INFO(this->get_logger(), "3 seconds have passed since the Driving signal, ");
        this->ready = true;
    }
}

void ControlP2::cleanUp()
{
    RCLCPP_INFO(this->get_logger(), "Cleaning up");

    this->ready = false;
    this->missionSet = false;
    
    if (this->control_timer) {
        this->control_timer->cancel();
    }

    if(this->control_manager->get_algorithm() != nullptr){
        RCLCPP_INFO(this->get_logger(), "Terminating algorithm");
        this->control_manager->terminate_algorithm();
    }

    lart_msgs::msg::DynamicsCMD cleanUpMailBox = lart_msgs::msg::DynamicsCMD();
    cleanUpMailBox.rpm = 0;
    cleanUpMailBox.steering_angle = 0.0;
    cleanUpMailBox.acc_cmd = 0.0;
    cleanUpMailBox.header.stamp = rclcpp::Clock().now();

    
    if(this->acc_mode){
        dynamics_torque_publisher->publish(cleanUpMailBox);
    }else{
        dynamics_rpm_publisher->publish(cleanUpMailBox);
    }

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlP2>());
    rclcpp::shutdown();
    return 0;
}

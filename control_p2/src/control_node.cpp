#include "control_p2/control_node.hpp"

using std::placeholders::_1;

ControlP2::ControlP2() : Node("control_node")
{

    /*------------------------------------------------------------------------------*/
    /*                                   PUBLISHERS                                 */
    /*------------------------------------------------------------------------------*/
    
    dynamics_publisher = this->create_publisher<lart_msgs::msg::DynamicsCMD>(TOPIC_DYNAMICS_CMD, 10);

    marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>(TOPIC_TARGET_MARKER, 10);


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
        TOPIC_DYNAMICS, 10, std::bind(&ControlP2::dynamics_callback, this, _1));

    state_subscriber = this->create_subscription<lart_msgs::msg::State>(
        TOPIC_STATE, 10, std::bind(&ControlP2::state_callback, this, _1));

    mission_subscriber = this->create_subscription<lart_msgs::msg::Mission>(
        TOPIC_MISSION, 10, std::bind(&ControlP2::mission_callback, this, _1));

    position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        TOPIC_SLAM, 10, std::bind(&ControlP2::pose_callback, this, _1));

    /*------------------------------------------------------------------------------*/
    /*                            CLASS INITIALIZATION                              */
    /*------------------------------------------------------------------------------*/
    control_manager = new ControlManager();

}

void ControlP2::state_callback(const lart_msgs::msg::State::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "State callback received: %d", msg->data);

    switch (msg->data)
    {
    case lart_msgs::msg::State::DRIVING:
        this->drivingSignalTimeStamp = msg->header.stamp;
        break;

    case lart_msgs::msg::State::FINISH:
        // Stop the car but inside the manager maybe
    case lart_msgs::msg::State::EMERGENCY:
        this->cleanUp();
        break;
    
    default:
        break;
    }
}

void ControlP2::mission_callback(const lart_msgs::msg::Mission::SharedPtr msg)
{

    RCLCPP_INFO(this->get_logger(), "Mission received: %d", msg->data);

    this->missionSet = true;

    float missionSpeed = 0.0;

    switch(msg->data){
        case lart_msgs::msg::Mission::SKIDPAD:
        case lart_msgs::msg::Mission::AUTOCROSS:
        case lart_msgs::msg::Mission::TRACKDRIVE:
            missionSpeed = DEFAULT_MAX_SPEED;
            break;
        case lart_msgs::msg::Mission::ACCELERATION:
            missionSpeed = ACC_SPEED;
            break;
        case lart_msgs::msg::Mission::EBS_TEST:
            missionSpeed = EBS_SPEED;
            break;
        default:
            break;
    }

    this->control_manager->set_missionSpeed(missionSpeed);
}

void ControlP2::path_callback(const lart_msgs::msg::PathSpline::SharedPtr msg)
{
    //Check if the car is ready to drive (3 seconds after the driving signal [FSG RULE BOOK 2026])
    if(this->ready == false){
        checkTimeStamp(msg->header.stamp);
    }
        
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
    this->control_manager->set_pose(*msg);
}

void ControlP2::dispatchDynamicsCMD()
{
    if(!this->ready || !this->missionSet){
        RCLCPP_WARN(this->get_logger(), "Control node not ready or mission not set, not sending commands");
        return;
    }
    // publish dynamics command
    lart_msgs::msg::DynamicsCMD control_output = this->control_manager->getDynamicsCMD();
    this->dynamics_publisher->publish(control_output);

    // publish target marker
    if(TARGET_MARKER_VISIBLE){
        visualization_msgs::msg::Marker target_marker = this->control_manager->get_target_marker();
        this->marker_publisher->publish(target_marker);
    }

    // log info
    if(LOG_INFO){
        this->control_manager->log_info();
    }
}

void ControlP2::checkTimeStamp(rclcpp::Time msgTimeStamp)
{
    rclcpp::Duration timeDiff = this->drivingSignalTimeStamp - msgTimeStamp;
    if (timeDiff.seconds() > 3.0)
    {
        RCLCPP_INFO(this->get_logger(), "3 seconds have passed since the Driving signal, the car can start");
        this->ready = true;
    }
}

void ControlP2::cleanUp()
{
    RCLCPP_INFO(this->get_logger(), "Cleaning up");
    lart_msgs::msg::DynamicsCMD cleanUpMailBox = lart_msgs::msg::DynamicsCMD();
    cleanUpMailBox.rpm = 0;
    cleanUpMailBox.steering_angle = 0.0;

    this->dynamics_publisher->publish(cleanUpMailBox);

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlP2>());
    rclcpp::shutdown();
    return 0;
}

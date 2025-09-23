#include "control_p2/control_node.hpp"

using std::placeholders::_1;

ControlP2::ControlP2() : Node("control_node")
{
    /*------------------------------------------------------------------------------*/
    /*                                    TOPICS                                    */
    /*------------------------------------------------------------------------------*/

    this->declare_parameter(PARAMS_TOPIC_PATH, "/planned_path_topic");
	this->get_parameter(PARAMS_TOPIC_PATH, path_topic);

    this->declare_parameter(PARAMS_TOPIC_SPEED, "/acu_origin/dynamics");
	this->get_parameter(PARAMS_TOPIC_SPEED, rpm_topic);

    this->declare_parameter(PARAMS_TOPIC_DYNAMICS_CMD, "pc_origin/dynamics");
	this->get_parameter(PARAMS_TOPIC_DYNAMICS_CMD, dynamics_cmd_topic);

    this->declare_parameter(PARAMS_TOPIC_STATE, "/pc_origin/system_status/critical_as/state");
    this->get_parameter(PARAMS_TOPIC_STATE, state_topic);

    this->declare_parameter(PARAMS_TOPIC_MISSION, "pc_origin/system_status/critical_as/mission");
    this->get_parameter(PARAMS_TOPIC_MISSION, mission_topic);

    this->declare_parameter(PARAMS_TOPIC_SLAM, "/ekf/state");
    this->get_parameter(PARAMS_TOPIC_SLAM, slam_topic);


    /*------------------------------------------------------------------------------*/
    /*                                   PUBLISHERS                                 */
    /*------------------------------------------------------------------------------*/
    
    dynamics_publisher = this->create_publisher<lart_msgs::msg::DynamicsCMD>(dynamics_cmd_topic, 10);

    /*------------------------------------------------------------------------------*/
    /*                                  SUBSCRIBERS                                 */
    /*------------------------------------------------------------------------------*/

    subscription_path = this->create_subscription<lart_msgs::msg::PathSpline>(
        path_topic, 10, std::bind(&SpacNode::path_callback, this, _1));

    subscription_speed = this->create_subscription<lart_msgs::msg::Dynamics>(
        current_speed_topic, 10, std::bind(&SpacNode::speed_callback, this, _1));

    state_subscriber = this->create_subscription<lart_msgs::msg::State>(
        state_topic, 10, std::bind(&SpacNode::state_callback, this, _1));

    mission_subscriber = this->create_subscription<lart_msgs::msg::Mission>(
        mission_topic, 10, std::bind(&SpacNode::mission_callback, this, _1));

    ekf_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        slam_topic, 10, std::bind(&SpacNode::ekf_callback, this, _1));

}

void ControlP2::state_callback(const lart_msgs::msg::State::SharedPtr msg)
{
    switch (msg->data)
    {
    case lart_msgs::msg::State::DRIVING:
        /* code */
        break;
    case lart_msgs::msg::State::FINISH:
    case lart_msgs::msg::State::EMERGENCY:
        this->cleanUp();
        break;
    
    default:
        break;
    }
}

void ControlP2::mission_callback(const lart_msgs::msg::Mission::SharedPtr msg)
{
    // mission logic for control
}

void ControlP2::path_callback(const lart_msgs::msg::PathSpline::SharedPtr msg)
{
    // save current path
}

void ControlP2::speed_callback(const lart_msgs::msg::Dynamics::SharedPtr msg)
{
    // save current speed
}

void ControlP2::dispatchDynamicsCMD()
{
    // publish dynamics command
}

void ControlP2::cleanUp()
{
    // Send message of zero speed and zero steering angle
    // Useful at the end of the program and emergency situations
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlP2>());
    rclcpp::shutdown();
    return 0;
}
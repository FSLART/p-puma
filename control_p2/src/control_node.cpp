#include "control_p2/control_node.hpp"

using std::placeholders::_1;

ControlP2::ControlP2() : Node("control_node")
{
    /*------------------------------------------------------------------------------*/
    /*                               FLAGS & PARAMETERS                             */
    /*------------------------------------------------------------------------------*/

    // ADD FLAGS GETTERS


    /*------------------------------------------------------------------------------*/
    /*                                   PUBLISHERS                                 */
    /*------------------------------------------------------------------------------*/
    
    dynamics_publisher = this->create_publisher<lart_msgs::msg::DynamicsCMD>(TOPIC_DYNAMICS_CMD, 10);

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
        this->control_manager->set_ready();
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

    RCLCPP_INFO(this->get_logger(), "Mission received: %d", msg->data);
    this->control_manager->set_mission(*msg);
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
    this->control_manager->set_pose(*msg);
}

void ControlP2::dispatchDynamicsCMD()
{
    // publish dynamics command
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


// ADICIONAR TESTES UNITARIOS !!!! VAI SER DO CARAÇAS COM UMA INTERFACE QUE E GENERICA !!!!!
#include "control_p2/control_node.hpp"

using std::placeholders::_1;

ControlP2::ControlP2() : Node("control_node")
{
    /*------------------------------------------------------------------------------*/
    /*                                    FLAGS                                     */
    /*------------------------------------------------------------------------------*/

    // ADD FLAGS GETTERS


    /*------------------------------------------------------------------------------*/
    /*                                   PUBLISHERS                                 */
    /*------------------------------------------------------------------------------*/
    
    dynamics_publisher = this->create_publisher<lart_msgs::msg::DynamicsCMD>(TOPIC_DYNAMICS_CMD, 10);

    /*------------------------------------------------------------------------------*/
    /*                                  SUBSCRIBERS                                 */
    /*------------------------------------------------------------------------------*/

    subscription_path = this->create_subscription<lart_msgs::msg::PathSpline>(
        TOPIC_PATH, 10, std::bind(&ControlP2::path_callback, this, _1));

    subscription_speed = this->create_subscription<lart_msgs::msg::Dynamics>(
        TOPIC_SPEED, 10, std::bind(&ControlP2::speed_callback, this, _1));

    state_subscriber = this->create_subscription<lart_msgs::msg::State>(
        TOPIC_STATE, 10, std::bind(&ControlP2::state_callback, this, _1));

    mission_subscriber = this->create_subscription<lart_msgs::msg::Mission>(
        TOPIC_MISSION, 10, std::bind(&ControlP2::mission_callback, this, _1));

    ekf_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        TOPIC_SLAM, 10, std::bind(&ControlP2::ekf_callback, this, _1));

}

void ControlP2::state_callback(const lart_msgs::msg::State::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "State callback received: %d", msg->data);

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

    RCLCPP_INFO(this->get_logger(), "Mission received: %d", msg->data);

    switch(msg->data){
        case lart_msgs::msg::Mission::ACCELERATION:
            this->target->set_mission(this->acc_speed);
            break;
        case lart_msgs::msg::Mission::EBS_TEST:
            this->target->set_mission(this->ebs_speed);
            break;
        default:
            break;
    }
}

void ControlP2::path_callback(const lart_msgs::msg::PathSpline::SharedPtr msg)
{
    // save current path
}

void ControlP2::speed_callback(const lart_msgs::msg::Dynamics::SharedPtr msg)
{
    // save current speed
}

void ControlP2::ekf_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // save current position from ekf
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
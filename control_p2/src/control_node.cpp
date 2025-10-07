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

    subscription_path = this->create_subscription<lart_msgs::msg::PathSpline>(
        TOPIC_PATH, 10, std::bind(&ControlP2::path_callback, this, _1));

    subscription_speed = this->create_subscription<lart_msgs::msg::Dynamics>(
        TOPIC_SPEED, 10, std::bind(&ControlP2::speed_callback, this, _1));

    state_subscriber = this->create_subscription<lart_msgs::msg::State>(
        TOPIC_STATE, 10, std::bind(&ControlP2::state_callback, this, _1));

    mission_subscriber = this->create_subscription<lart_msgs::msg::Mission>(
        TOPIC_MISSION, 10, std::bind(&ControlP2::mission_callback, this, _1));

    position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        TOPIC_SLAM, 10, std::bind(&ControlP2::pose_callback, this, _1));

    /*------------------------------------------------------------------------------*/
    /*                            CLASS INITIALIZATION                              */
    /*------------------------------------------------------------------------------*/
    target = new Target(/* PARAMETERS */);


    /*------------------------------------------------------------------------------*/
    /*                                VEHICLE MODEL                                 */
    /*------------------------------------------------------------------------------*/

    LoadVehicleConfig("config/vehicle_config.yaml", this->vehicle_config);

}

void ControlP2::state_callback(const lart_msgs::msg::State::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "State callback received: %d", msg->data);

    switch (msg->data)
    {
    case lart_msgs::msg::State::DRIVING:
        this->target->set_ready();
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
        case lart_msgs::msg::Mission::SKIDPAD:
        case lart_msgs::msg::Mission::AUTOCROSS:
        case lart_msgs::msg::Mission::TRACKDRIVE:
            this->target->set_maxSpeed(this->max_speed);
            break;
        case lart_msgs::msg::Mission::ACCELERATION:
            this->target->set_maxSpeed(this->acc_speed);
            break;
        case lart_msgs::msg::Mission::EBS_TEST:
            this->target->set_maxSpeed(this->ebs_speed);
            break;
        default:
            break;
    }
}

void ControlP2::path_callback(const lart_msgs::msg::PathSpline::SharedPtr msg)
{
    // save current path
    this->target->set_path(*msg);
}

void ControlP2::dynamics_callback(const lart_msgs::msg::Dynamics::SharedPtr msg)
{
    // save current speed
    this->target->set_dynamics(*msg);
}

void ControlP2::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // save current position from slam
    this->target->set_pose(*msg);
}

void ControlP2::dispatchDynamicsCMD()
{
    // publish dynamics command
}

void ControlP2::LoadVehicleConfig(std::string filepath, VehicleConfig &config)
{
    // Load vehicle configuration from YAML file
    std::ifstream file(filepath);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open vehicle config file: %s", filepath.c_str());
        return;
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
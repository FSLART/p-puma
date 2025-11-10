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

    position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        TOPIC_SLAM, 10, std::bind(&ControlP2::pose_callback, this, _1));

    /*------------------------------------------------------------------------------*/
    /*                            CLASS INITIALIZATION                              */
    /*------------------------------------------------------------------------------*/
    control_manager = new ControlManager();

    control_manager->set_missionSpeed(DEFAULT_MAX_SPEED);

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

    lart_msgs::msg::DynamicsCMD control_output = this->control_manager->getDynamicsCMD();

    // publish dynamics command
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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlP2>());
    rclcpp::shutdown();
    return 0;
}

#ifndef CONTROL_NODE_H_
#define CONTROL_NODE_H_

/*------------------------------------------------------------------------------*/
/*                                    INCULES                                   */
/*------------------------------------------------------------------------------*/

#include "rclcpp/rclcpp.hpp"
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "lart_msgs/msg/path_spline.hpp"
#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/mission.hpp"

/*------------------------------------------------------------------------------*/
/*                                    DEFINES                                   */
/*------------------------------------------------------------------------------*/

#define PARAMS_TOPIC_PATH "path_topic"
#define PARAMS_TOPIC_SPEED "current_speed_topic"
#define PARAMS_TOPIC_DYNAMICS_CMD "dynamics_cmd_topic"
#define PARAMS_TOPIC_STATE "state_topic"
#define PARAMS_TOPIC_MISSION "mission_topic"
#define PARAMS_TOPIC_SLAM "slam_topic"

class ControlP2 : public rclcpp::Node
{
public:

    ControlP2();

private:

    rclcpp::Publisher<lart_msgs::msg::DynamicsCMD>::SharedPtr dynamics_publisher;

    rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr state_subscriber;
    rclcpp::Subscription<lart_msgs::msg::Mission>::SharedPtr mission_subscriber;
    rclcpp::Subscription<lart_msgs::msg::PathSpline>::SharedPtr subscription_path;
    rclcpp::Subscription<lart_msgs::msg::Dynamics>::SharedPtr subscription_speed;

protected:
    // Subscribers
    std::string path_topic;
    std::string current_speed_topic;
    std::string state_topic;
    std::string mission_topic;
    std::string slam_topic;

    // Publishers
    std::string dynamics_cmd_topic;

    // Callbacks
    void ControlP2::state_callback(const lart_msgs::msg::State::SharedPtr msg);
    void ControlP2::mission_callback(const lart_msgs::msg::Mission::SharedPtr msg);
    void ControlP2::path_callback(const lart_msgs::msg::PathSpline::SharedPtr msg);
    void ControlP2::speed_callback(const lart_msgs::msg::Dynamics::SharedPtr msg);

    // Functions
    void ControlP2::dispatchDynamicsCMD();
    void ControlP2::cleanUp();  

};

#endif
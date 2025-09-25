#ifndef CONTROL_NODE_H_
#define CONTROL_NODE_H_

/*------------------------------------------------------------------------------*/
/*                                    INCLUDES                                   */
/*------------------------------------------------------------------------------*/

#include "topics.hpp"

#include "rclcpp/rclcpp.hpp"
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "lart_msgs/msg/path_spline.hpp"
#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/mission.hpp"



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

    // Callbacks
    void ControlP2::state_callback(const lart_msgs::msg::State::SharedPtr msg);
    void ControlP2::mission_callback(const lart_msgs::msg::Mission::SharedPtr msg);
    void ControlP2::path_callback(const lart_msgs::msg::PathSpline::SharedPtr msg);
    void ControlP2::speed_callback(const lart_msgs::msg::Dynamics::SharedPtr msg);
    void ControlP2::ekf_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // Functions
    void ControlP2::dispatchDynamicsCMD();
    void ControlP2::cleanUp();  

};

#endif
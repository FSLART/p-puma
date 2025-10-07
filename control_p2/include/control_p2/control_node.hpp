#ifndef CONTROL_NODE_H_
#define CONTROL_NODE_H_

/*------------------------------------------------------------------------------*/
/*                                   INCLUDES                                   */
/*------------------------------------------------------------------------------*/

#include "target.hpp"
#include "rclcpp/rclcpp.hpp"



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
    void state_callback(const lart_msgs::msg::State::SharedPtr msg);
    void mission_callback(const lart_msgs::msg::Mission::SharedPtr msg);
    void path_callback(const lart_msgs::msg::PathSpline::SharedPtr msg);
    void dynamics_callback(const lart_msgs::msg::Dynamics::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // Functions
    void dispatchDynamicsCMD();
    void cleanUp();

    //Class
    Target *target;

    // Parameters
    VehicleConfig vehicle_config;

};

#endif
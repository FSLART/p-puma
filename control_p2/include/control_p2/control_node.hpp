#ifndef CONTROL_NODE_H_
#define CONTROL_NODE_H_

/*------------------------------------------------------------------------------*/
/*                                   INCLUDES                                   */
/*------------------------------------------------------------------------------*/

#include "control_manager.hpp"
#include "rclcpp/rclcpp.hpp"



class ControlP2 : public rclcpp::Node
{
public:

    ControlP2();

    // Callbacks
    void state_callback(const lart_msgs::msg::State::SharedPtr msg);
    void mission_callback(const lart_msgs::msg::Mission::SharedPtr msg);
    void path_callback(const lart_msgs::msg::PathSpline::SharedPtr msg);
    void dynamics_callback(const lart_msgs::msg::Dynamics::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void cleanUp();

private:

    rclcpp::Publisher<lart_msgs::msg::DynamicsCMD>::SharedPtr dynamics_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;

    rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr state_subscriber;
    rclcpp::Subscription<lart_msgs::msg::Mission>::SharedPtr mission_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscriber;
    rclcpp::Subscription<lart_msgs::msg::PathSpline>::SharedPtr path_subscriber;
    rclcpp::Subscription<lart_msgs::msg::Dynamics>::SharedPtr dynamics_subscriber;

protected:

    // Functions
    void dispatchDynamicsCMD();
    void checkTimeStamp(rclcpp::Time msgTimeStamp);

    // Parameters
    bool ready = false;
    bool missionSet = false;
    rclcpp::Time drivingSignalTimeStamp;
    rclcpp::TimerBase::SharedPtr control_timer;

    // flags
    bool race_finished = false;

    //Class
    ControlManager *control_manager;

};

#endif
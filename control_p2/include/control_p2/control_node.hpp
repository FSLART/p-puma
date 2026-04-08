#ifndef CONTROL_NODE_H_
#define CONTROL_NODE_H_

/*------------------------------------------------------------------------------*/
/*                                   INCLUDES                                   */
/*------------------------------------------------------------------------------*/

#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "control_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include <optional>
#include <chrono>



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
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &params);
    void cleanUp();

private:

    rclcpp::Publisher<lart_msgs::msg::DynamicsCMD>::SharedPtr dynamics_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;

    rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr state_subscriber;
    rclcpp::Subscription<lart_msgs::msg::Mission>::SharedPtr mission_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscriber;
    rclcpp::Subscription<lart_msgs::msg::PathSpline>::SharedPtr path_subscriber;
    rclcpp::Subscription<lart_msgs::msg::Dynamics>::SharedPtr dynamics_subscriber;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

protected:

    // Functions
    void dispatchDynamicsCMD();
    void checkTimeStamp();

    // ROS Parameters
    bool sim_mode;
    bool log_info;
    bool target_marker_visible;
    float default_max_speed;
    float acc_speed;
    float ebs_speed;
    float lookahead_time;
    float tau;
    float kp;
    float ki;
    float kd;

    // Parameters
    bool ready = false;
    bool missionSet = false;
    std::optional<std::chrono::steady_clock::time_point> drivingSignalTimeStamp;
    rclcpp::TimerBase::SharedPtr control_timer;

    // flags
    bool race_finished = false;

    //Class
    ControlManager *control_manager;

};

#endif
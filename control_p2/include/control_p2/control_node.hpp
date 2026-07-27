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
#include "std_msgs/msg/float32.hpp"



class ControlP2 : public rclcpp::Node
{
public:

    ControlP2();

    // Callbacks
    void state_callback(const lart_msgs::msg::State::SharedPtr msg);
    void mission_callback(const lart_msgs::msg::Mission::SharedPtr msg);
    void path_callback(const lart_msgs::msg::PathArray::SharedPtr msg);
    void dynamics_callback(const lart_msgs::msg::Dynamics::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void lap_callback(const lart_msgs::msg::SlamStats::SharedPtr msg);
    void final_path_callback(const lart_msgs::msg::PathArray::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &params);
    void cleanUp();

private:

    rclcpp::Publisher<lart_msgs::msg::DynamicsCMD>::SharedPtr dynamics_rpm_publisher;
    rclcpp::Publisher<lart_msgs::msg::DynamicsCMD>::SharedPtr dynamics_torque_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lookahead_publisher;

    rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr state_subscriber;
    rclcpp::Subscription<lart_msgs::msg::Mission>::SharedPtr mission_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscriber;
    rclcpp::Subscription<lart_msgs::msg::PathArray>::SharedPtr path_subscriber;
    rclcpp::Subscription<lart_msgs::msg::PathArray>::SharedPtr final_path_subscriber;
    rclcpp::Subscription<lart_msgs::msg::Dynamics>::SharedPtr dynamics_subscriber;
    rclcpp::Subscription<lart_msgs::msg::SlamStats>::SharedPtr lap_subscriber;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

protected:

    // Functions
    void dispatchDynamicsCMD();
    void checkTimeStamp();

    // ROS Parameters
    bool sim_mode;
    bool log_info;
    bool target_marker_visible;
    bool fsl_flag;
    bool acc_mode;
    float fsl_speed;
    float default_max_speed;
    float acc_speed;
    float skidpad_speed;
    float ebs_speed;
    float lookahead_time;
    float tau;
    float kv;
    float curvature_gain;
    float kp;
    float ki;
    float kd;

    // Parameters
    bool ready = false;
    bool missionSet = false;
    bool finalPathReceived = false;
    uint8_t current_mission;
    std::optional<std::chrono::steady_clock::time_point> drivingSignalTimeStamp;
    rclcpp::TimerBase::SharedPtr control_timer;
    float distance_after_finish = 0.0f;
    rclcpp::Time finish_time;

    // flags
    bool race_finished = false;

    //Class
    ControlManager *control_manager;

};

#endif
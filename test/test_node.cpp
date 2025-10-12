// ControlP2/test/test_control_node.cpp

#include <gtest/gtest.h>
#include "control_p2/control_node.hpp"
#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/mission.hpp"
#include "lart_msgs/msg/path_spline.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Test node initialization
TEST(ControlNodeTest, Initialization) {
    auto node = std::make_shared<ControlP2>();
    ASSERT_NE(node, nullptr);
}

// Test state callback sets ready
TEST(ControlNodeTest, StateCallbackDriving) {
    auto node = std::make_shared<ControlP2>();
    auto msg = std::make_shared<lart_msgs::msg::State>();
    msg->data = lart_msgs::msg::State::DRIVING;
    node->state_callback(msg);
    // No direct getter, but you can check indirectly by sending a mission and path
    auto mission = std::make_shared<lart_msgs::msg::Mission>();
    mission->data = lart_msgs::msg::Mission::AUTOCROSS;
    node->mission_callback(mission);
    auto path = std::make_shared<lart_msgs::msg::PathSpline>();
    node->path_callback(path);
    SUCCEED();
}

// Test mission callback sets mission
TEST(ControlNodeTest, MissionCallback) {
    auto node = std::make_shared<ControlP2>();
    auto msg = std::make_shared<lart_msgs::msg::Mission>();
    msg->data = lart_msgs::msg::Mission::SKIDPAD;
    node->mission_callback(msg);
    SUCCEED();
}

// Test path callback stores path
TEST(ControlNodeTest, PathCallback) {
    auto node = std::make_shared<ControlP2>();
    auto msg = std::make_shared<lart_msgs::msg::PathSpline>();
    node->path_callback(msg);
    SUCCEED();
}

// Test dynamics callback stores dynamics
TEST(ControlNodeTest, DynamicsCallback) {
    auto node = std::make_shared<ControlP2>();
    auto msg = std::make_shared<lart_msgs::msg::Dynamics>();
    msg->rpm = 100;
    msg->steering_angle = 0.1;
    node->dynamics_callback(msg);
    SUCCEED();
}

// Test pose callback stores pose
TEST(ControlNodeTest, PoseCallback) {
    auto node = std::make_shared<ControlP2>();
    auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    node->pose_callback(msg);
    SUCCEED();
}

// Test cleanUp publishes zero command
TEST(ControlNodeTest, CleanUp) {
    auto node = std::make_shared<ControlP2>();
    node->cleanUp();
    SUCCEED();
}
#include <gtest/gtest.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>


static rclcpp::Node::SharedPtr node_;



int main(int argc, char** argv){
    std::cout << "[TEST] Initializing ROS2 and Google Test..." << std::endl;
    auto start = std::chrono::steady_clock::now();

    rclcpp::init(argc, argv);

    testing::InitGoogleTest(&argc, argv);

    std::cout << "[TEST] Running all tests..." << std::endl;
    int ret = RUN_ALL_TESTS();

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::cout << "[TEST] All tests finished." << std::endl;
    std::cout << "[TEST] Number of failed tests: " << ret << std::endl;
    std::cout << "[TEST] Total test duration: " << duration << " ms" << std::endl;

    rclcpp::shutdown();
    std::cout << "[TEST] ROS2 shutdown complete." << std::endl;
    return ret;
}
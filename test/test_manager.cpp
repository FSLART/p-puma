#include <gtest/gtest.h>
#include "control_p2/control_manager.hpp"
#include "lart_msgs/msg/path_array.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "lart_msgs/msg/mission.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

/*------------------------------------------------------------------------------*/
/*                               TEST SETTER                                    */
/*------------------------------------------------------------------------------*/

// Test setting path, dynamics, and pose
TEST(ControlManagerTest, Setters) {
    ControlManager manager;
    lart_msgs::msg::PathArray path;
    lart_msgs::msg::Dynamics dynamics;
    geometry_msgs::msg::PoseStamped pose;
    manager.set_path(path);
    manager.set_dynamics(dynamics);
    manager.set_pose(pose);
    // No direct getters, but no crash means success
    SUCCEED();
}

/*------------------------------------------------------------------------------*/
/*                             TEST GETTERS                                     */
/*------------------------------------------------------------------------------*/


// Test getDynamicsCMD returns valid command when ready and mission set
TEST (tst_manager, GetStraightPathAngle){
    lart_msgs::msg::PathArray path;
    lart_msgs::msg::Dynamics dynamics;
    geometry_msgs::msg::PoseStamped pose_stamped;

    //Expected values
    float expected_steering_angle = 0.0;

    //Generating Data
    dynamics.rpm = 0;
    dynamics.steering_angle = 0.0;


    std::vector<std::vector<float>> data = {
    {0.0, 0.0, 0, 0.03},
    {0.5, 0.5, 0, 0.03},
    {1.0, 1.0, 0, 0.03},
    {1.5, 1.5, 0, 0.03},
    {2.0, 2.0, 0, 0.03},
    {2.5, 2.5, 0, 0.03},
    {3.0, 3.0, 0, 0.03},
    {3.5, 3.5, 0, 0.03},
    {4.0, 4.0, 0, 0.03},
    {4.5, 4.5, 0, 0.03},
    {5.0, 5.0, 0, 0.03},
    {5.5, 5.5, 0, 0.03}
    };

    // Populate the PathArray message
    for (const auto& row : data) {
        float distance = row[0];
        float x = row[1];
        float y = row[2];
        float curvature = row[3];

        // Create and populate a PathPoint
        lart_msgs::msg::PathPoint point;
        point.x = x;
        point.y = y;
        point.curvature = curvature;
        point.distance = distance;

        // Add the PathPoint to the path
        path.points.push_back(point);

        // Keep pose_stamped in sync with the last point for set_pose
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        pose_stamped.pose.position.z = 0.0; // Assume z = 0 for 2D path
        pose_stamped.pose.orientation.w = 1.0; // Default orientation (no rotation)
    }
    ControlManager controlManager;

    // Set up the control manager with the test data
    controlManager.initialize_algorithm(5.0, 0.5, 0.1, 0.1,0.1, 0.2, 0.05, 0.01);
    controlManager.set_path(path);
    controlManager.set_dynamics(dynamics);
    controlManager.set_pose(pose_stamped);

    lart_msgs::msg::DynamicsCMD controlOutput = controlManager.getDynamicsCMD();

    std::cerr << "[          ] steering angle = " << controlOutput.steering_angle << std::endl;
    ASSERT_NEAR(controlOutput.steering_angle, expected_steering_angle, 0.01);

}


// Test getDynamicsCMD when an 0 curvature is provided
TEST (tst_manager, get_straight_angle){
    lart_msgs::msg::PathArray path;
    lart_msgs::msg::Dynamics dynamics;
    geometry_msgs::msg::PoseStamped pose_stamped;

    //Expected values
    float expected_steering_angle = 0.0;

    //Generating Data
    dynamics.rpm = 0;
    dynamics.steering_angle = 0.0;

    std::vector<std::vector<float>> data = {
    {0.0, 0.0, 0, 0.0},
    {0.5, 0.5, 0, 0.0},
    {1.0, 1.0, 0, 0.0},
    {1.5, 1.5, 0, 0.0},
    {2.0, 2.0, 0, 0.0},
    {2.5, 2.5, 0, 0.0},
    {3.0, 3.0, 0, 0.0},
    {3.5, 3.5, 0, 0.0},
    {4.0, 4.0, 0, 0.0},
    {4.5, 4.5, 0, 0.0},
    {5.0, 5.0, 0, 0.0},
    {5.5, 5.5, 0, 0.0}
    };

    // Populate the PathArray message
    for (const auto& row : data) {
        float distance = row[0];
        float x = row[1];
        float y = row[2];
        float curvature = row[3];

        // Create and populate a PathPoint
        lart_msgs::msg::PathPoint point;
        point.x = x;
        point.y = y;
        point.curvature = curvature;
        point.distance = distance;

        // Add the PathPoint to the path
        path.points.push_back(point);

        // Keep pose_stamped in sync with the last point for set_pose
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        pose_stamped.pose.position.z = 0.0; // Assume z = 0 for 2D path
        pose_stamped.pose.orientation.w = 1.0; // Default orientation (no rotation)
    }
    ControlManager controlManager;

    // Set up the control manager with the test data
    controlManager.initialize_algorithm(5.0, 0.5, 0.1, 0.1,0.1, 0.2, 0.05, 0.01);
    controlManager.set_path(path);
    controlManager.set_dynamics(dynamics);
    controlManager.set_pose(pose_stamped);

    lart_msgs::msg::DynamicsCMD controlOutput = controlManager.getDynamicsCMD();

    std::cerr << "[          ] steering angle = " << controlOutput.steering_angle << std::endl;
    ASSERT_NEAR(controlOutput.steering_angle, expected_steering_angle, 0.01);
}

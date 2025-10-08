#ifndef TARGET_H_
#define TARGET_H_

#include "options.hpp"
#include "math/pure_pursuit.hpp"

class ControlManager {
    public: 
        ControlManager();
        void set_ready();
        void set_path(lart_msgs::msg::PathSpline path);
        void set_dynamics(lart_msgs::msg::Dynamics dynamics);
        void set_pose(geometry_msgs::msg::PoseStamped pose);
        void set_mission(lart_msgs::msg::Mission mission);
        lart_msgs::msg::DynamicsCMD getDynamicsCMD();

    private:
        Pursuit_Algorithm algorithm;
    protected:
        // Parameters
        bool ready = false;
        bool missionSet = false;
        uint8_t currentSpeed;
        float currentSteering;
        geometry_msgs::msg::PoseStamped currentPose;
        lart_msgs::msg::PathSpline currentPath;
        float missionSpeed;

    
};

#endif
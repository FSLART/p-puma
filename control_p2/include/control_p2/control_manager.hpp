#ifndef TARGET_H_
#define TARGET_H_

#include "options.hpp"
#include ALGORITHM

class ControlManager {
    public: 
        ControlManager(float mission_speed);
        void set_ready();
        void set_maxSpeed(float speed);
    private:
        Pursuit_Algorithm algorithm;
    protected:
        // Parameters
        bool ready = false;
        bool missionSet = false;
        uint8_t currentSpeed;
        float currentSteering;

    
};

#endif
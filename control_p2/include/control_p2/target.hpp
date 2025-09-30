#ifndef TARGET_H_
#define TARGET_H_

#include "math/pure_pursuit.hpp"
#include "utils.hpp"


class Target{
    public:
        Target(float mission_speed);
        void set_ready();
        void set_maxSpeed(float speed);
    private:
        Pursuit_Algorithm algorithm;
    protected:
        // Parameters
        float missionSpeed;
        bool ready = false;
        bool missionSet = false;
        uint8_t currentSpeed;
        float currentSteering;

    
};

#endif
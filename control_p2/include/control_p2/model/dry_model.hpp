#ifndef DRY_MODEL_H_
#define DRY_MODEL_H_

#include "model/vehicle_config.hpp"  // Include VehicleConfig

class DryModel : public VehicleConfig {
    public:
        DryModel(): VehicleConfig() {
            // Set dry-specific parameters
            this->set_grip_coefficient(1.9f);
            this->set_max_speed(30.0f);  // Example value for dry conditions
        }
};

#endif
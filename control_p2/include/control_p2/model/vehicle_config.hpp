#ifndef VEHICLE_CONFIG_H_
#define VEHICLE_CONFIG_H_

class VehicleConfig {
    private:
        // Physical Dimensions
        float wheelbase;                    // Distance between front and rear axles [m]
        float track_width_front;            // Front track width (wheel to wheel) [m]
        float track_width_rear;             // Rear track width (wheel to wheel) [m]
        float tire_radius;                  // Tire radius [m]

        // Mass Properties
        float total_mass;                   // Total vehicle mass [kg]
        float cg_height;                    // Center of gravity height [m]
        float cg_to_front_axle;            // Distance from CG to front axle [m]
        float cg_to_rear_axle;             // Distance from CG to rear axle [m]

        // Performance Parameters
        float grip_coefficient;             // Tire-road friction coefficient
        float max_steering_angle;           // Maximum steering angle [rad]
        float max_acceleration;             // Maximum acceleration [m/s^2]
        float max_deceleration;             // Maximum deceleration [m/s^2]
        float max_speed;   
        
    protected:
        // Setters
        void set_wheelbase(float value) { wheelbase = value; }
        void set_track_width_front(float value) { track_width_front = value; }
        void set_track_width_rear(float value) { track_width_rear = value; }
        void set_tire_radius(float value) { tire_radius = value; }
        void set_total_mass(float value) { total_mass = value; }
        void set_cg_height(float value) { cg_height = value; }
        void set_cg_to_front_axle(float value) { cg_to_front_axle = value; }
        void set_cg_to_rear_axle(float value) { cg_to_rear_axle = value; }
        void set_grip_coefficient(float value) { grip_coefficient = value; }
        void set_max_steering_angle(float value) { max_steering_angle = value; }
        void set_max_acceleration(float value) { max_acceleration = value; }
        void set_max_deceleration(float value) { max_deceleration = value; }
        void set_max_speed(float value) { max_speed = value; }

    public:
        // Constructor
        VehicleConfig();

        // Getters
        float get_wheelbase() const { return wheelbase; }
        float get_track_width_front() const { return track_width_front; }
        float get_track_width_rear() const { return track_width_rear; }
        float get_tire_radius() const { return tire_radius; }
        float get_total_mass() const { return total_mass; }
        float get_cg_height() const { return cg_height; }
        float get_cg_to_front_axle() const { return cg_to_front_axle; }
        float get_cg_to_rear_axle() const { return cg_to_rear_axle; }
        float get_grip_coefficient() const { return grip_coefficient; }
        float get_max_steering_angle() const { return max_steering_angle; }
        float get_max_acceleration() const { return max_acceleration; }
        float get_max_deceleration() const { return max_deceleration; }
        float get_max_speed() const { return max_speed; }

        // Setters
        
};

#endif
#ifndef PP_PROFILE_H_
#define PP_PROFILE_H_

/*
 * pp_profile — Pure Pursuit + planned-profile longitudinal control.
 *
 * Lateral control is IDENTICAL to uff_pursuit (velocity-proportional lookahead
 * clamp(lookahead_time*v, MIN, MAX) + Euclidean-chord pure pursuit). Only the
 * LONGITUDINAL path changes:
 *
 *   uff_pursuit  : v_target = sqrt(mu*g*kv / kappa_bar)   (reactive)
 *                  acc_cmd  = PID(v_target - v)            (feedback only)
 *
 *   pp_profile   : v_target = path.velocity[0]            (planned, looked up)
 *                  a_ff     = (v[k]^2 - v[0]^2)/(2*k*ds)  (energy form)
 *                  acc_cmd  = a_ff/(mu_long*g) + PID(...) (feedforward + feedback)
 *                  + combined-signal anti-windup
 *
 * DEPENDENCY (step C): requires lart_msgs/PathSpline to carry a per-point
 * `velocity[]` array parallel to `curvature[]`, filled by the path provider.
 * Until that field exists this file will not compile when selected.
 *
 * To activate (after step C lands):
 *   - utils.hpp   : #define ALGORITHM "math/pp_profile.hpp"
 *   - CMakeLists  : src/math/uff_pursuit.cpp -> src/math/pp_profile.cpp
 */

#include "../utils.hpp"

#include <algorithm>
#include <optional>
#include <cmath>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;

class PID_Controller {
    public:
        PID_Controller() = default;
        PID_Controller(float kp, float ki, float kd);

        // Feedforward-aware PID with COMBINED-signal anti-windup.
        // ff_cmd is the already-normalized feedforward command. Because ff and
        // fb sum before the +/-1 saturation, watching only the PID output is
        // insufficient: integration is frozen when the combined command would
        // saturate AND the error pushes further into saturation. Returns fb_cmd.
        float compute(float setpoint, float input, float dt, float ff_cmd);

        void set_P(float kp);
        void set_I(float ki);
        void set_D(float kd);
        void reset();

    protected:
        float kp, ki, kd;
        float error_prev, error_sum;
};

class Pursuit_Algorithm {
    public:
        Pursuit_Algorithm(float missionSpeed, float lookahead_time, float tau, float kv, float curvature_gain, float kp, float ki, float kd);
        lart_msgs::msg::DynamicsCMD calculate_control(lart_msgs::msg::PathSpline path, geometry_msgs::msg::PoseStamped pose,
             float current_speed, float current_steering);

        geometry_msgs::msg::PoseStamped get_target_point();
        void set_missionSpeed(float missionSpeed);
        void set_lookahead_time(float lookahead_time);
        void set_tau(float tau);
        void set_kv(float kv);
        void set_curvature_gain(float curvature_gain);
        void set_kp(float kp);
        void set_ki(float ki);
        void set_kd(float kd);
        float calculate_lookahead(float speed);     // public: ControlManager calls it for logging
    private:
        // functions
        int fastRound(float x);
        float lowPassFilter(float input, float dt);
        float feedforward_accel(const lart_msgs::msg::PathSpline& path);   // energy-form a_ff at the car

        // Parameters
        lart_msgs::msg::DynamicsCMD prevOutput;
        rclcpp::Time prevTime;

        int closest_point_index = -1;
        geometry_msgs::msg::PoseStamped target_point;
        float missionSpeed;
        float lookahead_time;
        float tau;
        float kv;                 // unused (no reactive speed) — kept for interface parity
        float curvature_gain;     // unused (no curvature lookahead) — kept for interface parity
        VehicleModel vehicle = VehicleModel();

        // PID Controller
        PID_Controller pid_controller;
};

#endif

#ifndef UTILS_H_
#define UTILS_H_

/*------------------------------------------------------------------------------*/
/*                              ALGORITHM & MODEL                               */
/*------------------------------------------------------------------------------*/

#define ALGORITHM  "math/control_algorithm.hpp"
#define MODEL "model/dry_model.hpp"


/*------------------------------------------------------------------------------*/
/*                                   INCLUDES                                   */
/*------------------------------------------------------------------------------*/

//Custom msgs
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "lart_msgs/msg/path_spline.hpp"
#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/mission.hpp"
#include "lart_msgs/msg/slam_stats.hpp"
#include "lart_msgs/msg/path_array.hpp"

// Common values
#include "lart_common.h"
#include "topics.h"

// Vehicle Config
#include MODEL

/*------------------------------------------------------------------------------*/
/*                                   TOPICS                                     */
/*------------------------------------------------------------------------------*/

// #define TOPIC_PATH_OLD "/planned_path_topic"
// #define TOPIC_DYNAMICS_OLD "/acu_origin/dynamics"
// #define TOPIC_DYNAMICS_CMD "pc_origin/dynamics"
// #define TOPIC_STATE_OLD "/pc_origin/system_status/critical_as/state"
// #define TOPIC_MISSION_OLD "/pc_origin/system_status/critical_as/mission"
// #define TOPIC_SLAM_OLD "/slam/pose"
// #define TOPIC_SLAM_STATS_OLD "/slam/stats"
//#define TOPIC_TARGET_MARKER "/target_marker_topic"

/*------------------------------------------------------------------------------*/
/*                                  CONSTANTS                                   */
/*------------------------------------------------------------------------------*/

// La gravedad
#define LART_GRAVITY 9.81f

#define DEFAULT_IMU_TO_REAR_AXLE 1.15f

#define PATH_SIZE 100

// LOOKAHEAD PARAMETERS
#define MAX_LOOKAHEAD 8.0f
#define MIN_LOOKAHEAD 4.5f

#define SPACE_BETWEEN_POINTS 0.10f
#define MIN_TARGET_INDEX MIN_LOOKAHEAD/SPACE_BETWEEN_POINTS

#define SIZE_AVG_ARRAY 3

#define FREQUENCY 50 // Hz

#define MAX_RPM_DELTA 10.0f
#define MAX_MS_DELTA 0.05f

// PID CONTROLLER SIGNAL LIMITS
// acc_cmd is the fraction of max motor current (VCU torque target -1..1).
// On the ground 1.0 is ~13-20 m/s2 below ~15 m/s (Milagres 13-10 manual
// runs), so +-0.5 still allows ~7 m/s2 while capping a runaway loop.
#define MIN_SIG_VAL -0.5f
#define MAX_SIG_VAL 0.5f

// SPEED SETPOINT RAMP [m/s2]: the PID tracks a setpoint that moves towards
// the desired speed at most this fast, so the acceleration does not depend
// on the loop gain (or on the car being on the ground or in the air).
#define SPEED_RAMP_UP 2.0f
#define SPEED_RAMP_DOWN 3.0f

// ACC_CMD SLEW LIMIT [fraction of max current per second]: acc_cmd can rise
// at most this fast (0 -> 0.5 takes 0.5 s, ~15 m/s3 on the ground) and fall
// faster so releasing/braking is not delayed much. Safety net on top of the
// speed ramp, catches steps from param changes, re-init or path jumps.
#define ACC_CMD_SLEW_UP 1.0f
#define ACC_CMD_SLEW_DOWN 3.0f

// Upper bound on the control period used by the PID/ramp [s]; avoids a large
// integral/ramp jump on the first call or after a stall.
#define MAX_CONTROL_DT 0.1f


#endif
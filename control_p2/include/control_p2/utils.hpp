#ifndef UTILS_H_
#define UTILS_H_

/*------------------------------------------------------------------------------*/
/*                              ALGORITHM & MODEL                               */
/*------------------------------------------------------------------------------*/

#define ALGORITHM  "math/sacc_pursuit.hpp"
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
#define MIN_LOOKAHEAD 3.5f

#define SPACE_BETWEEN_POINTS 0.10f
#define MIN_TARGET_INDEX MIN_LOOKAHEAD/SPACE_BETWEEN_POINTS

#define SIZE_AVG_ARRAY 3

#define FREQUENCY 50 // Hz

#define MAX_RPM_DELTA 3.0f
#define MAX_MS_DELTA 0.05f

// PID CONTROLLER SIGNAL LIMITS
#define MIN_SIG_VAL -1.0f
#define MAX_SIG_VAL 1.0f


#endif
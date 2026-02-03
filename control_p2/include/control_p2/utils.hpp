#ifndef UTILS_H_
#define UTILS_H_

/*------------------------------------------------------------------------------*/
/*                                   INCLUDES                                   */
/*------------------------------------------------------------------------------*/

//OPTIONS
#include "options.hpp"

//Custom msgs
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "lart_msgs/msg/path_spline.hpp"
#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/mission.hpp"

// Common values
#include "lart_common.h"

// Vehicle Config
#include MODEL

/*------------------------------------------------------------------------------*/
/*                                   TOPICS                                     */
/*------------------------------------------------------------------------------*/

#define TOPIC_PATH "/planned_path_topic"
#define TOPIC_DYNAMICS "/acu_origin/dynamics"
#define TOPIC_DYNAMICS_CMD "pc_origin/dynamics"
#define TOPIC_STATE "/pc_origin/system_status/critical_as/state"
#define TOPIC_MISSION "/pc_origin/system_status/critical_as/mission"
#define TOPIC_SLAM "/ekf/state"
#define TOPIC_TARGET_MARKER "/target_marker_topic"

/*------------------------------------------------------------------------------*/
/*                                  CONSTANTS                                   */
/*------------------------------------------------------------------------------*/

// La gravedad
#define LART_GRAVITY 9.81f

#define DEFAULT_IMU_TO_REAR_AXLE 1.15f

// LOOKAHEAD PARAMETERS
#define MAX_LOOKAHEAD 8.5f
#define MIN_LOOKAHEAD 3.6f

#define SPACE_BETWEEN_POINTS 0.50f
#define MIN_TARGET_INDEX MIN_LOOKAHEAD/SPACE_BETWEEN_POINTS

#define SIZE_AVG_ARRAY 3

// PID CONTROLLER SIGNAL LIMITS
#define MIN_SIG_VAL -1.0f
#define MAX_SIG_VAL 1.0f


#endif
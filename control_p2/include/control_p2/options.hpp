#ifndef OPTIONS_H_
#define OPTIONS_H_



/*                /$$$$$$  /$$$$$$$  /$$$$$$$$ /$$$$$$  /$$$$$$  /$$   /$$  /$$$$$$                   */
/*               /$$__  $$| $$__  $$|__  $$__/|_  $$_/ /$$__  $$| $$$ | $$ /$$__  $$                  */
/*              | $$  \ $$| $$  \ $$   | $$     | $$  | $$  \ $$| $$$$| $$| $$  \__/                  */
/*              | $$  | $$| $$$$$$$/   | $$     | $$  | $$  | $$| $$ $$ $$|  $$$$$$                   */
/*              | $$  | $$| $$____/    | $$     | $$  | $$  | $$| $$  $$$$ \____  $$                  */
/*              | $$  | $$| $$         | $$     | $$  | $$  | $$| $$\  $$$ /$$  \ $$                  */
/*              |  $$$$$$/| $$         | $$    /$$$$$$|  $$$$$$/| $$ \  $$|  $$$$$$/                  */
/*               \______/ |__/         |__/   |______/ \______/ |__/  \__/ \______/                   */



/*------------------------------------------------------------------------------*/
/*                              ALGORITHM & MODEL                               */
/*------------------------------------------------------------------------------*/

#pragma region Algorithm & Model

#define ALGORITHM  "math/lp_pursuit.hpp"
#define MODEL "model/dry_model.hpp"

#pragma endregion
/*------------------------------------------------------------------------------*/
/*                       MAXIMUM SPEED FOR EACH MISSION                         */
/*------------------------------------------------------------------------------*/

#pragma region Maximum Speed for Each Mission

#define DEFAULT_MAX_SPEED 2.0f
#define ACC_SPEED 2.0f
#define EBS_SPEED 2.0f

#pragma endregion
/*------------------------------------------------------------------------------*/
/*                           PUBLISHER FREQUENCY (HZ)                           */
/*------------------------------------------------------------------------------*/

#pragma region Publisher Frequency (Hz)

#define FREQUENCY 50 // Hz

#pragma endregion
/*------------------------------------------------------------------------------*/
/*                              ALGORITHM TUNING                                */
/*------------------------------------------------------------------------------*/

#pragma region Lookahead Time

#define LOOKAHEAD_TIME 0.5f // seconds
#define TAU 0.01f // seconds for low pass filter
#define MAX_RPM_DELTA 3.0f // maximum change in RPM per control cycle

#define KP 1.0f
#define KI 0.1f
#define KD 0.05f

#pragma endregion
/*------------------------------------------------------------------------------*/
/*                                 DEBUG FLAGS                                  */
/*------------------------------------------------------------------------------*/

#pragma region Debug Flags

#define TARGET_MARKER_VISIBLE true
#define LOG_INFO false

#pragma endregion

#endif
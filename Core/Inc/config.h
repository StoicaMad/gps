/*
 * config.h
 *
 *  Created on: Oct 3, 2025
 *      Author: stoica
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#pragma once

#define CONTROL_HZ             10

// GPS gating
#define SOG_LOCK_MIN_MPS       0.55f
#define COG_STD_MAX_DEG        5.0f

// Distanțe (m)
#define ARRIVAL_RADIUS_M       1.0f
#define APPROACH_DIST_M        8.0f
#define GEOFENCE_RADIUS_M      200.0f

// PID heading (start)
#define PID_KP_INIT            1.0f
#define PID_KI_INIT            0.05f
#define PID_KD_INIT            0.20f
#define RUDDER_LIMIT_DEG       28.0f
#define RUDDER_RATE_DPS        80.0f
#define HEADING_DB_TRACK_DEG   2.0f
#define HEADING_DB_APP_DEG     5.0f

typedef enum { DRIVE_PERIAT=0, DRIVE_BRUSHLESS=1 } DriveProfile;

// iBUS channels (adaptează la tine)
#define CH_SWITCH_MODE     5   // SWC: 0=manual, 1=goto, 2=home
#define CH_VRA             6   // VRA -> Kp
#define CH_VRB             7   // VRB -> Kd


#endif /* INC_CONFIG_H_ */

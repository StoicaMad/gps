/*
 *  state_machine.h
 *
 *  Created on: Oct 3, 2025
 *      Author: stoica
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_


// state_machine.h
#pragma once
typedef enum { AP_IDLE=0, AP_HEADING_LOCKED, AP_LAUNCH, AP_TRACK, AP_ARRIVED } APState;
typedef struct { APState state; float heading_used_deg, last_rel_heading_deg; } APContext;
void ap_init(APContext* ctx, float initial_heading);
void ap_step(APContext* ctx,
             double curLat,double curLon, double tgtLat,double tgtLon,
             float sog_mps, float cog_deg, float cog_std_deg,
             int driveProfile,
             float* rudder_out_deg, float* throttle_target_mps);



#endif /* INC_STATE_MACHINE_H_ */

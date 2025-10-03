/*
 * state_machine.c
 *
 *  Created on: Oct 3, 2025
 *      Author: stoica
 */


// state_machine.c
#include "state_machine.h"
#include "config.h"
#include "nav_math.h"
#include "pid_heading.h"
#include "gnss.h"

static PIDHeading pid;
static float cruise_periat=1.3f, cruise_brush=1.0f; // m/s

void ap_init(APContext* c,float hdg){
  c->state = AP_HEADING_LOCKED; c->heading_used_deg = hdg; c->last_rel_heading_deg = hdg;
  pid_init(&pid, PID_KP_INIT, PID_KI_INIT, PID_KD_INIT, 1.0f/CONTROL_HZ);
  pid_limits(&pid, -RUDDER_LIMIT_DEG, RUDDER_LIMIT_DEG, RUDDER_RATE_DPS);
}

void ap_step(APContext* c, double clat,double clon,double tlat,double tlon,
             float sog,float cog,float cog_std, int driveProfile,
             float* rudder_deg, float* target_sog){

  float bearing=0, dist=0; ll_to_bearing_dist(clat,clon,tlat,tlon,&bearing,&dist);
  bool cogStable = (cog_std <= COG_STD_MAX_DEG);
  coglock_update(cog, sog, cogStable, &c->heading_used_deg, &c->last_rel_heading_deg);

  switch(c->state){
    case AP_HEADING_LOCKED:
      *target_sog = 0.0f; *rudder_deg=0.0f;
      if (dist > ARRIVAL_RADIUS_M) c->state = AP_LAUNCH;
      break;

    case AP_LAUNCH:
      *target_sog = (driveProfile==DRIVE_PERIAT)?0.8f:0.6f;
      *rudder_deg = 0.0f;
      if (sog >= SOG_LOCK_MIN_MPS && cogStable) { pid_reset(&pid); c->state = AP_TRACK; }
      break;

    case AP_TRACK:{
      float cruise = (driveProfile==DRIVE_PERIAT)?cruise_periat:cruise_brush;
      *target_sog = (dist > APPROACH_DIST_M) ? cruise : 0.25f + (cruise-0.25f)*(dist/APPROACH_DIST_M);
      pid_deadband(&pid, (dist > APPROACH_DIST_M)?HEADING_DB_TRACK_DEG:HEADING_DB_APP_DEG);
      float err = wrap180(bearing - c->heading_used_deg);
      *rudder_deg = pid_step(&pid, err, c->heading_used_deg);
      if (dist <= ARRIVAL_RADIUS_M){ *target_sog = 0.0f; *rudder_deg=0.0f; c->state=AP_ARRIVED; }
    } break;

    case AP_ARRIVED:
      *target_sog = 0.0f; *rudder_deg = 0.0f; c->state=AP_HEADING_LOCKED; break;

    default: *target_sog=0; *rudder_deg=0; c->state=AP_HEADING_LOCKED; break;
  }
}

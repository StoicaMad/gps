/*
 * pid_heading.h
 *
 *  Created on: Oct 3, 2025
 *      Author: stoica
 */

#ifndef INC_PID_HEADING_H_
#define INC_PID_HEADING_H_

#pragma once

typedef struct {
  float kp, ki, kd, dt;        // parametrii PID
  float i;                     // integral
  float prev_meas;             // ultima măsurătoare
  float d_lp;                  // filtrul derivativ low-pass
  float out_min, out_max;      // limite output (ex: -500..+500 µs)
  float rate_lim;              // limită de viteză (µs/s)
  float deadband_us;           // zonă moartă (±µs)
} PIDHeading;

// Inițializare PID
void  pid_init(PIDHeading* p, float kp, float ki, float kd, float dt);

// Setare limite ieșire și rată
void  pid_limits(PIDHeading* p, float out_min, float out_max, float rate_lim);

// Setare deadband (±µs)
void  pid_deadband(PIDHeading* p, float db_us);

// Reset integrator
void  pid_reset(PIDHeading* p);

// Un pas PID (returnează offset µs)
float pid_step(PIDHeading* p, float error_us, float meas_us);


#endif /* INC_PID_HEADING_H_ */

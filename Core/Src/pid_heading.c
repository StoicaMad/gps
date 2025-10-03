/*
 * pid_heading.c
 *
 *  Created on: Oct 3, 2025
 *      Author: stoica
 */


#include <math.h>
#include "pid_heading.h"
#include "nav_math.h"


void pid_init(PIDHeading* p, float kp, float ki, float kd, float dt){
  p->kp = kp;
  p->ki = ki;
  p->kd = kd;
  p->dt = dt;
  p->i = 0;
  p->prev_meas = 0;
  p->d_lp = 0;
  p->out_min = -500;   // ±500 µs
  p->out_max =  500;
  p->rate_lim = 200;   // µs/s
  p->deadband_us = 0;
}

void pid_limits(PIDHeading* p, float mn, float mx, float rate){
  p->out_min = mn;
  p->out_max = mx;
  p->rate_lim = rate;
}

void pid_deadband(PIDHeading* p, float db){
  p->deadband_us = db;
}

void pid_reset(PIDHeading* p){
  p->i = 0;
  p->prev_meas = 0;
  p->d_lp = 0;
}

float pid_step(PIDHeading* p, float err_us, float meas_us){
  // Deadband
  if (fabsf(err_us) < p->deadband_us) err_us = 0.0f;

  // Integral
  p->i += p->ki * err_us * p->dt;
  if (p->i > p->out_max) p->i = p->out_max;
  if (p->i < p->out_min) p->i = p->out_min;

  // Derivativ (filtrat)
  float dmeas = (meas_us - p->prev_meas) / p->dt;
  p->prev_meas = meas_us;
  float alpha = 0.2f;
  p->d_lp = alpha * (-dmeas) + (1.0f - alpha) * p->d_lp;

  // PID output
  float u = p->kp * err_us + p->i + p->kd * p->d_lp;

  // Limitare output
  if (u > p->out_max) u = p->out_max;
  if (u < p->out_min) u = p->out_min;

  return u; // offset în µs
}


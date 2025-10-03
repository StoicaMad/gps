/*
 * speed_profile.c
 *
 *  Created on: Oct 3, 2025
 *      Author: stoica
 */


// speed_profile.c
#include "speed_profile.h"
#include "config.h"
float sog_target_to_pwm(float target_mps, float cur_pwm, float dt, int profile){
    float max_mps = (profile==DRIVE_PERIAT)?2.0f:1.6f;

    if (target_mps < 0) {
        target_mps = 0;
    }
    if (target_mps > max_mps) {
        target_mps = max_mps;
    }

    float pwm_target = target_mps / max_mps; // 0..1
    float slew = 0.5f * dt; // pwm/s

    if (pwm_target > cur_pwm) {
        cur_pwm = (pwm_target - cur_pwm > slew) ? (cur_pwm + slew) : pwm_target;
    } else if (pwm_target < cur_pwm) {
        cur_pwm = (cur_pwm - pwm_target > slew) ? (cur_pwm - slew) : pwm_target;
    }

    if (cur_pwm < 0) {
        cur_pwm = 0;
    }
    if (cur_pwm > 1) {
        cur_pwm = 1;
    }

    return cur_pwm;
}

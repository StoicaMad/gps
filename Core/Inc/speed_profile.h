/*
 * speed_profile.h
 *
 *  Created on: Oct 3, 2025
 *      Author: stoic
 */

#ifndef INC_SPEED_PROFILE_H_
#define INC_SPEED_PROFILE_H_

// speed_profile.h
#pragma once
float sog_target_to_pwm(float target_mps, float cur_pwm, float dt, int profile);


#endif /* INC_SPEED_PROFILE_H_ */

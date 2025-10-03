/*
 * pwm_driver.h
 *
 *  Created on: Oct 3, 2025
 *      Author: stoic
 */

#ifndef INC_PWM_DRIVER_H_
#define INC_PWM_DRIVER_H_

#include <stdint.h>

void pwm_init(void);
void pwm_set_servo_us(uint16_t us);
void pwm_set_motor_us(uint16_t us);

#endif /* INC_PWM_DRIVER_H_ */

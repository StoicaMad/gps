/*
 * pwm_driver.c
 *
 *  Created on: Oct 3, 2025
 *      Author: stoica
 */


#include "pwm_driver.h"
#include "tim.h"   // pentru htim3 și TIM_CHANNEL_x
#include "stm32f1xx_hal.h"

void pwm_init(void){
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1); // ESC
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2); // Servo
}

// Setează direct valoarea PWM în µs (1000–2000 tipic)
void pwm_set_servo_us(uint16_t us){
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, us);
}

void pwm_set_motor_us(uint16_t us){
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, us);
}


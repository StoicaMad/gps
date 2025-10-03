/*
 * circular_filter.h
 *
 *  Created on: Oct 3, 2025
 *      Author: stoica
 */

#ifndef INC_CIRCULAR_FILTER_H_
#define INC_CIRCULAR_FILTER_H_


// circular_filter.h
#pragma once
typedef struct { float alpha, s, c; } CircularEMA;
void  circEMA_init(CircularEMA* f, float alpha, float initial_deg);
float circEMA_update(CircularEMA* f, float sample_deg);
float circ_mean_deg(float* buf_deg, int n);
float circ_std_deg (float* buf_deg, int n);


#endif /* INC_CIRCULAR_FILTER_H_ */

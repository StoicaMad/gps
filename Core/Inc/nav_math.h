/*
 * nav_math.h
 *
 *  Created on: Oct 3, 2025
 *      Author: stoica
 */

#ifndef INC_NAV_MATH_H_
#define INC_NAV_MATH_H_


// nav_math.h
#pragma once
float wrap180(float deg);
float wrap360(float deg);
void  ll_to_bearing_dist(double lat1, double lon1, double lat2, double lon2,
                         float* bearing_deg, float* distance_m);


#endif /* INC_NAV_MATH_H_ */

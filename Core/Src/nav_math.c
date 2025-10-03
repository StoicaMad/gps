/*
 * nav_math.c
 *
 *  Created on: Oct 3, 2025
 *      Author: stoic
 */


// nav_math.c
#include <math.h>
#include "nav_math.h"
static inline double rad(double d){ return d*M_PI/180.0; }
static inline double deg(double r){ return r*180.0/M_PI; }
float wrap360(float a){ while(a<0) a+=360; while(a>=360) a-=360; return a; }
float wrap180(float a){ a=fmodf(a+180.0f,360.0f); if(a<0) a+=360.0f; return a-180.0f; }
void ll_to_bearing_dist(double lat1,double lon1,double lat2,double lon2,
                        float* bearing_deg,float* dist_m){
  double φ1=rad(lat1), φ2=rad(lat2);
  double Δφ=rad(lat2-lat1), Δλ=rad(lon2-lon1);
  double a = sin(Δφ/2)*sin(Δφ/2) + cos(φ1)*cos(φ2)*sin(Δλ/2)*sin(Δλ/2);
  double c = 2*atan2(sqrt(a), sqrt(1-a));
  if(dist_m) *dist_m = (float)(6371000.0 * c);
  double y = sin(Δλ)*cos(φ2);
  double x = cos(φ1)*sin(φ2) - sin(φ1)*cos(φ2)*cos(Δλ);
  double θ = atan2(y, x);
  if(bearing_deg) *bearing_deg = (float)wrap360((float)deg(θ));
}

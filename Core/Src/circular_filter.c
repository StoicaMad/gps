/*
 * circular_filter.c
 *
 *  Created on: Oct 3, 2025
 *      Author: stoica
 */


// circular_filter.c
#include <math.h>
#include "circular_filter.h"
static inline float radf(float d){ return d*(float)M_PI/180.0f; }
static inline float degf(float r){ return r*180.0f/(float)M_PI; }
void circEMA_init(CircularEMA* f,float alpha,float initial_deg){
  f->alpha=alpha; f->s=sinf(radf(initial_deg)); f->c=cosf(radf(initial_deg));
}
float circEMA_update(CircularEMA* f,float deg){
  float sn=sinf(radf(deg)), cs=cosf(radf(deg));
  f->s = f->alpha*sn + (1.0f-f->alpha)*f->s;
  f->c = f->alpha*cs + (1.0f-f->alpha)*f->c;
  return degf(atan2f(f->s, f->c));
}
float circ_mean_deg(float* x,int n){
  float S=0,C=0; for(int i=0;i<n;i++){ S+=sinf(radf(x[i])); C+=cosf(radf(x[i])); }
  return degf(atan2f(S/n, C/n));
}
float circ_std_deg(float* x,int n){
  float S=0,C=0; for(int i=0;i<n;i++){ S+=sinf(radf(x[i])); C+=cosf(radf(x[i])); }
  float R = sqrtf((S*S + C*C))/(float)n; if (R <= 1e-6f) return 180.0f;
  return degf(sqrtf(-2.0f*logf(R)));
}

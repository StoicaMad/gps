/*
 * cog_stability.c
 *
 *  Created on: Oct 3, 2025
 *      Author: stoica
 */


// cog_stability.c
#include <stdlib.h>
#include <stdbool.h>
#include "circular_filter.h"
static float* buf; static int N, idx, filled;
void cogstab_init(int window_len){ N=window_len; idx=0; filled=0; buf=(float*)malloc(sizeof(float)*N); }
bool cogstab_update(float x,float* std_deg){
  if (!buf) return false;
  buf[idx]=x; idx=(idx+1)%N; if(filled<N) filled++;
  if(filled<4){ if(std_deg)*std_deg=999; return false; }
  float s = circ_std_deg(buf, filled); if(std_deg) *std_deg = s; return true;
}

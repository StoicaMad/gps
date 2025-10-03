/*
 * gnss.h
 *
 *  Created on: Oct 3, 2025
 *      Author: stoica
 */

#ifndef INC_GNSS_H_
#define INC_GNSS_H_


// gnss.h
#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef struct {
  double lat, lon;      // deg
  float  sog_mps;       // m/s
  float  cog_deg;       // 0..360
  float  hdop;
  uint8_t fixType;      // 0..5
  uint32_t last_ms;
} GNSSFix;

void   gnss_init(uint32_t baud);
bool   gnss_poll(GNSSFix* out);   // non-blocking; true dacă avem update
bool   gnss_good(const GNSSFix* f);

// COG lock
void   coglock_reset(float initial_deg);
float  coglock_update(float cog_deg, float sog_mps, bool cog_stable,
                      float* heading_used_deg, float* last_reliable_deg);



#endif /* INC_GNSS_H_ */

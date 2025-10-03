/*
 * cog_stability.h
 *
 *  Created on: Oct 3, 2025
 *      Author: stoica
 */

#ifndef INC_COG_STABILITY_H_
#define INC_COG_STABILITY_H_

// cog_stability.h
#pragma once
#include <stdbool.h>
void  cogstab_init(int window_len);
bool  cogstab_update(float new_cog_deg, float* std_out_deg);


#endif /* INC_COG_STABILITY_H_ */

/*!****************************************************************************
 * @file		linearinterp.h
 * @author		Storozhenko Roman - D_EL
 * @version 	V1.0.0
 * @date		22-01-2022
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */

#ifndef LINEARINTERP_H
#define LINEARINTERP_H

#include <stdint.h>

/*!****************************************************************************
 * @brief
 */
static inline int32_t iqs32_Fy_x1x2y1y2x(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x){
	int32_t df = y1-y0;
	int32_t dx = x1-x0;
	if(dx == 0){
		return INT32_MAX;
	}
	return y0 + ((int64_t)df * ((x - x0))) / dx;
}

#endif /* LINEARINTERP_H */

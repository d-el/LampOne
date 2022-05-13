/*!****************************************************************************
 * @file		movingAverageFilter.h
 * @author		Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date		22-01-2022
 * @copyright 	The MIT License (MIT). Copyright (c) 2022 Storozhenko Roman
 */

#ifndef MOVINGAVERAGEFILTER_H
#define MOVINGAVERAGEFILTER_H

#include <stdint.h>
#include <stddef.h>

template<class T, size_t MaxSize>
class MovingAverageFilter {
public:
	MovingAverageFilter(T _defValue, size_t _size = MaxSize){
		size = _size;
		for(size_t i = 0; i < size; i++){
			buffer[i] = _defValue;
		}
	};

	T proc(T val){
		acc -= buffer[index];
		buffer[index] = val;
		acc += buffer[index];

		index++;
		if(index >= size){
			index = 0;
		}

		return acc / (int32_t)size;
	}

private:
	size_t size;
	size_t index = 0;
	T buffer[MaxSize];
	decltype(buffer[0]*buffer[0]) acc;

};

#endif /* MOVINGAVERAGEFILTER_H */

/******************************** END OF FILE ********************************/

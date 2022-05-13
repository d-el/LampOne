/*!****************************************************************************
 * @file		measurer.h
 * @author		Storozhenko Roman - D_EL
 * @version 	V1.0.0
 * @date		22-01-2022
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */

#ifndef MEASURER_H_
#define MEASURER_H_

template<class T>
class Measurer{
public:
	Measurer(T a = 0, T b = 0):
			a(a), b(b){
	}

	void calibrate(T x0, T y0, T x1, T y1){
		a = (y1 - y0) / (x1 - x0);
		b = y0 - a * x0;
	}

	T getValue(T x){
		return a * x + b;
	}

private:
	T a, b;
};

#endif /* MEASURER_H_ */

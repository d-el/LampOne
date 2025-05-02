/*!****************************************************************************
 * @file		adcTSK.c
 * @author		Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date		17-01-2022
 * @copyright 	The MIT License (MIT). Copyright (c) 2022 Storozhenko Roman
 */

/*!****************************************************************************
 * Include
 */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <adc.h>
#include <pwm.h>
#include <gpio.h>
#include "adcTSK.h"
#include <movingAverageFilter.h>

/*!****************************************************************************
 * MEMORY
 */
adcTaskStct_type adcTaskStct;
int32_t tpwm, tsigma;

/*!****************************************************************************
 *
 */
template<int32_t _resolution, int32_t _maxValue>
class sigma_delta_modulator{
public:
	static constexpr int32_t resolution = _resolution;
	static constexpr inline int32_t max_val = _maxValue;

	int32_t operator()(int32_t in){
		int y = acc >> resolution;

		if(y < 0){
			y = 0;
		}
		if(y > max_val){
			y = max_val;
		}

		acc += in - (y << resolution);
		return y;
	};

private:
	int32_t acc = 0;;
};

/*!****************************************************************************
 *
 */
class PI{
public:
	PI(int32_t _Kp, int32_t _Ki, int64_t _integral_max):
		Kp(_Kp), Ki(_Ki), integral_max(_integral_max)
	{};

	int32_t operator()(int32_t ref, int32_t v)
	{
		int64_t err = ref - v;

		integral += err;

		if(integral > integral_max){
			integral = integral_max;
		}
		if(integral < 0){
			integral = 0;
		}

		i_val = Ki * integral;

		int32_t out = (Kp * err + i_val) >> 12;

		if(out < 0)
			out = 0;
		return out;
	};

private:
	int32_t Kp;
	int32_t Ki;

	int32_t p_val = 0;
	int32_t i_val = 0;
	int64_t integral = 0;
	int64_t integral_max;
};

/*!****************************************************************************
 * @brief
 * @param
 * @retval
 */
void adcTSK(void *pPrm){
	(void)pPrm;
	adcTaskStct_type& a = adcTaskStct;
	static sigma_delta_modulator<7, 410> modulator;
	static MovingAverageFilter<uint16_t, 64> f_vin(0);
	static MovingAverageFilter<uint16_t, 16> f_iled1(0);
	static MovingAverageFilter<uint16_t, 64> f_temperature(0);
	static MovingAverageFilter<uint16_t, 32> f_vref(0);
	PI pi = PI(400, 350, INT32_MAX/10);

	static SemaphoreHandle_t AdcEndConversionSem;
	vSemaphoreCreateBinary(AdcEndConversionSem);
	xSemaphoreTake(AdcEndConversionSem, portMAX_DELAY);

	auto pwmUpdateHandler = [](){ // PWM interrupt handler
		tsigma = modulator(tpwm);
		pwm1set(tsigma);
	};

	pwm_UpdateCallbackSet(pwmUpdateHandler);
	pwm_init();

	static adcStct_type adcValue;
	auto adcHoock = [](adcStct_type *adc){ // ADC interrupt handler
		adcValue = *adc;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(AdcEndConversionSem, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	};

	adc_setCallback(adcHoock);
	adc_setSampleRate(100);
	adc_init();
	adc_startSampling();

	int32_t devider = 150;
	int32_t startup = 100;
	int16_t temperatureSampleDivider = 0;

	while(1){
		xSemaphoreTake(AdcEndConversionSem, portMAX_DELAY);

		a.filtered.vin = f_vin.proc(adcValue.adcreg[CH_UINADC]);
		a.filtered.iled1 = f_iled1.proc(adcValue.adcreg[CH_ILED1] * 8);
		if(startup == 0 || temperatureSampleDivider++ >= 3000){
			temperatureSampleDivider = 0;
			a.filtered.temperature = f_temperature.proc(adcValue.adcreg[CH_TEMPERATURE] * 8);
		}
		a.filtered.vref = f_vref.proc(adcValue.adcreg[CH_VREF]);

		if(startup == 0){
			if(--devider == 0){
				tpwm = pi(a.targetcurrent, a.filtered.iled1);
				devider = 32;
			}
		}

		if(startup > 0){
			startup--;
		}
		if(startup == 1){
			a.iled1offset = a.filtered.iled1;
		}
	}
}

/******************************** END OF FILE ********************************/

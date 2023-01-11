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
#include <pid.h>
#include <pwm.h>
#include <gpio.h>
#include "adcTSK.h"
#include <movingAverageFilter.h>

/*!****************************************************************************
 * Local function declaration
 */
static void adcHoock(adcStct_type *adc);

/*!****************************************************************************
 * MEMORY
 */
adcTaskStct_type adcTaskStct;
SemaphoreHandle_t AdcEndConversionSem;
adcStct_type adcValue;

int32_t Kp = 8000;
int32_t Ki = 250;
int32_t Kd = 200;

uint32_t tpwm = 20;

/*!****************************************************************************
 * @brief
 * @param
 * @retval
 */
void adcTSK(void *pPrm){
	(void)pPrm;
	adcTaskStct_type& a = adcTaskStct;

	vSemaphoreCreateBinary(AdcEndConversionSem);
	xSemaphoreTake(AdcEndConversionSem, portMAX_DELAY);

	adc_setCallback(adcHoock);
	adc_setSampleRate(100);
	adc_init();
	adc_startSampling();

	static MovingAverageFilter<uint16_t, 64> f_vin(0);
	static MovingAverageFilter<uint16_t, 128> f_iled1(0);
	static MovingAverageFilter<uint16_t, 1> f_iled1fast(0);
	static MovingAverageFilter<uint16_t, 32> f_temperature(0);
	static MovingAverageFilter<uint16_t, 32> f_vref(0);

	uint8_t qn = 22;
	Pid::PID pidController = Pid::PID(0, Kp, Ki, Kd, qn, Pid::feedbackPositive, Pid::proportionalToError);
	pidController.init(16, -2144316798);
	pidController.setOutputMin(0);
	pidController.setOutputMax(400);

	uint16_t led1targetprev = 0;
	uint32_t error = 0;

	while(1){
		xSemaphoreTake(AdcEndConversionSem, portMAX_DELAY);

		a.filtered.vin = f_vin.proc(adcValue.adcreg[CH_UINADC]);
		a.filtered.iled1 = f_iled1.proc(adcValue.adcreg[CH_ILED1] * 8);
		uint16_t iled1fast = f_iled1fast.proc(adcValue.adcreg[CH_ILED1] * 8);
		a.filtered.temperature = f_temperature.proc(adcValue.adcreg[CH_TEMPERATURE]);
		a.filtered.vref = f_vref.proc(adcValue.adcreg[CH_VREF]);

		if(a.targetcurrent == 0){
			gppin_reset(GP_EN1);
			pwm1set(0);
		}
		if(led1targetprev == 0 && a.targetcurrent > 0){
			pwm1set(0);
			vTaskDelay(10);
			error = 0;
			gppin_set(GP_EN1);
		}
		led1targetprev = a.targetcurrent;

		if(a.targetcurrent > 0 && error == 0){
			pidController.setSetpoint(a.targetcurrent);
			tpwm = pidController.compute(iled1fast);
			pwm1set(tpwm);
		}

		// Update PID settings
		if(pidController.getKp() != Kp || pidController.getKi() != Ki || pidController.getKd() != Kd){
			pidController.setTunings(Kp, Ki, Kd);
		}
	}
}

/*!****************************************************************************
 * @brief
 */
static void adcHoock(adcStct_type *adc){
	adcValue = *adc;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(AdcEndConversionSem, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/******************************** END OF FILE ********************************/

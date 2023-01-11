/*!****************************************************************************
 * @file		systemTSK.c
 * @author		Storozhenko Roman - D_EL
 * @version		V1.0
 * @date		11.05.2022
 * @copyright 	The MIT License (MIT). Copyright (c) 2022 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include <assert.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <adc.h>
#include <pwm.h>
#include <flash.h>
#include <board.h>
#include <prmSystem.h>
#include <printp.h>
#include <prmSystem.h>
#include "systemTSK.h"
#include "modbusTSK.h"
#include "adcTSK.h"
#include <arm_math.h>
#include <linearinterp.h>
#include <movingAverageFilter.h>

#include <uart.h>

// MCU calibration data
#define CAL_VREF_DATA	(*(uint16_t*)0x1FFF75AA)	// ADC value VREF_INT at 3.0V VREF
#define CAL_TS_DATA		(*(uint16_t*)0x1FFF75A8)	// ADC value Temperature sensot at 30 °C 3.0V VREF
#define TS_LINEARITY	2.5							// mv / °C

/*!****************************************************************************
* Memory
*/
extern uint8_t _suser_settings;
bool flagsave;

static const uint16_t brightness2current[200] = {
	11,	11,	11,	11,	11,	11,	11,	11,	11,	11,	11,	12,	12,	12,	12,	12,
	12,	12,	12,	12,	12,	12,	12,	12,	12,	12,	13,	13,	13,	13,	13,	13,
	13,	13,	13,	13,	14,	14,	14,	14,	14,	14,	14,	15,	15,	15,	15,	15,
	15,	16,	16,	16,	16,	16,	17,	17,	17,	17,	18,	18,	18,	18,	19,	19,
	19,	20,	20,	20,	21,	21,	22,	22,	22,	23,	23,	24,	24,	25,	25,	26,
	26,	27,	28,	28,	29,	29,	30,	31,	32,	32,	33,	34,	35,	36,	36,	37,
	38,	39,	40,	41,	43,	44,	45,	46,	47,	49,	50,	51,	53,	54,	56,	58,
	59,	61,	63,	65,	67,	69,	71,	73,	75,	77,	80,	82,	85,	87,	90,	93,
	96,	99,	102,	105,	108,	112,	115,	119,	123,	127,	131,	135,	139,	144,	149,	153,
	159,	164,	169,	175,	181,	186,	193,	199,	206,	213,	220,	227,	235,	243,	251,	259,
	268,	277,	286,	296,	306,	316,	327,	338,	350,	362,	374,	387,	400,	414,	428,	443,
	458,	474,	490,	507,	524,	542,	561,	580,	600,	621,	642,	664,	687,	711,	736,	761,
	788,	815,	843,	872,	903,	934,	966,	1000
};

/*!****************************************************************************
 * @brief
 */
bool savePrm(void){
	//return true;
	size_t settingsize = Prm::getSerialSize(Prm::savesys);
	uint8_t settingbuf[((settingsize + 7) / 8) * 8];
	Prm::serialize(Prm::savesys, settingbuf);
	flash_unlock();
	flash_erasePage(&_suser_settings);
	flashState_type memState = flash_write(&_suser_settings, settingbuf, sizeof(settingbuf));
	flash_lock();
	if(memState != flash_ok){
		return false;
	}
	return true;
}

void loadPrm(void){
	auto settingsize = Prm::getSerialSize(Prm::savesys);
	if(!Prm::deserialize(Prm::savesys, &_suser_settings, settingsize)){
		__NOP();
	}
}

void setCurrent(Prm::Val<uint16_t>& prm, bool read, void *arg){
	(void)arg;
	(void)prm;
	if(read){
		return;
	}
	Prm::adcCurrent.val = adcTaskStct.filtered.iled1;
}

/*!****************************************************************************
 * @brief
 */
void systemTSK(void *pPrm){
	(void)pPrm;

	adcTaskStct_type &a = adcTaskStct;

	loadPrm();
	assert(pdTRUE == xTaskCreate(modbusTSK, "modbusTSK", MODBUS_TSK_SZ_STACK,  NULL, MODBUS_TSK_PRIO, NULL));
	assert(pdTRUE == xTaskCreate(adcTSK, "adcTSK", ADC_TSK_SZ_STACK, NULL, ADC_TSK_PRIO, NULL));

	uint16_t btBrightnessIndex = 0;
	for(size_t i = 0; i < sizeof(brightness2current)/sizeof(brightness2current[0]) - 1; i++){
		if(Prm::setcurrent.val >= brightness2current[i] && Prm::setcurrent.val <= brightness2current[i + 1]){
			int16_t one = Prm::setcurrent.val - brightness2current[i];
			int16_t two = brightness2current[i + 1] - Prm::setcurrent.val;
			btBrightnessIndex = one < two ? i : i + 1;
			break;
		}
	}

	static MovingAverageFilter<uint16_t, 120> currentTagetFilter(0);
	bool bypassFilter = false;
	bool outen = Prm::enableout.val ? true : false;
	bool up = btBrightnessIndex == sizeof(brightness2current)/sizeof(brightness2current[0]) - 1 ? false : true;
	TickType_t pxPreviousWakeTime = xTaskGetTickCount();
	while(1){
		// Button discrete filter
		static uint16_t discreteCnt;
		constexpr uint16_t discreteTh = 5;
		static bool discreteOut;
		if(!gppin_get(GP_BT1)){
			if(discreteCnt < discreteTh)
				discreteCnt++;
			else
				discreteOut = true;
		}
		else{
			if(discreteCnt > 0)
				discreteCnt--;
			else
				discreteOut = false;
		}

		// Long press detect
		static uint16_t longCnt = 0;
		constexpr uint16_t longTh = 20;
		static bool longOut;
		if(discreteOut){
			if(longCnt < longTh)
				longCnt++;
			else
				longOut = true;
		}
		else{
			longCnt = 0;
			longOut = false;
		}

		// Edge detector
		static bool discreteOutPrev;
		bool falling = false;
		if(!discreteOut && discreteOutPrev){
			falling = true;
		}
		discreteOutPrev = discreteOut;

		// On/Off
		static bool longOutPrev;
		if(falling && !longOutPrev){
			outen = !outen;

			Prm::enableout.val = outen ? 1 : 0;
			savePrm();
			bypassFilter = false;
		}

		if(longOutPrev && !longOut){
			savePrm();
		}

		longOutPrev = longOut;

		// Up/Down
		if(falling /*&& outen*/){
			up = !up;
		}
		if(longOut && outen){
			if(up){
				if(btBrightnessIndex < sizeof(brightness2current)/sizeof(brightness2current[0]) - 1) btBrightnessIndex += 1;
			}
			else{
				if(btBrightnessIndex > 0) btBrightnessIndex -= 1;
			}
			Prm::setcurrent.val = brightness2current[btBrightnessIndex];
		}

		outen ? LED_OFF() : LED_ON();

		// Calc ADC Vref
		uint16_t vref = ((uint32_t)3000 * CAL_VREF_DATA) / a.filtered.vref;

		// Calc temperature
		int32_t tsCal = (CAL_TS_DATA * a.filtered.vref) / CAL_VREF_DATA;
		int32_t y = tsCal + (4095 * TS_LINEARITY * 100) / vref;	// ADC value at 130 °C
		Prm::temperature.val = iqs32_Fy_x1x2y1y2x(	tsCal,
												300,						// 30.0 °C
												y,
												1300,						// 130.0 °C
												a.filtered.temperature);

		// Calc Current
		Prm::current.val = iqs32_Fy_x1x2y1y2x(0, 30, Prm::adcCurrent.val, Prm::с_current.val, a.filtered.iled1);

		// Calc target in ADC LSB value
		uint16_t targetAdcLsb = iqs32_Fy_x1x2y1y2x(0, 0, Prm::с_current.val, Prm::adcCurrent.val, Prm::setcurrent.val);

		// Calc input voltage
		constexpr int32_t Rh = 100, Rl = 2;
		Prm::input_voltage.val = (a.filtered.vin * vref * (Rh + Rl)) / (4096 * Rl);

		// Termal compensation
		constexpr int16_t termalThreshpoint = 800; // X_X °C
		int16_t termalCompensation = 0;
		if(Prm::temperature.val > termalThreshpoint){
			termalCompensation = (Prm::temperature.val - termalThreshpoint) * 50;
			if(termalCompensation > targetAdcLsb) termalCompensation = targetAdcLsb;
			bypassFilter = false;
		}
		uint16_t targetcurrent = outen ? targetAdcLsb - termalCompensation : 0;
		uint16_t outfilter = currentTagetFilter.proc(targetcurrent);
		if(outfilter < brightness2current[0]) outfilter = 0;
		a.targetcurrent = bypassFilter ? targetcurrent : outfilter;
		if(targetcurrent == outfilter) bypassFilter = true;

		vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(SYSTEM_TSK_PERIOD));
	}
}

/*!****************************************************************************
*
*/
void OSinit(void){
	BaseType_t res = xTaskCreate(systemTSK, "systemTSK", SYSTEM_TSK_SZ_STACK, NULL, SYSTEM_TSK_PRIO, NULL);
	assert(res == pdTRUE);
	vTaskStartScheduler();
}

/******************************** END OF FILE ********************************/

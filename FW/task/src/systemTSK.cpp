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

static const uint16_t brightness2current[150] = {
		1,      1,      1,      1,      1,      1,      1,      1,      2,      2,      2,      2,      2,      2,      2,      2,
		2,      2,      2,      3,      3,      3,      3,      3,      3,      3,      3,      4,      4,      4,      4,      4,
		5,      5,      5,      5,      5,      6,      6,      6,      7,      7,      7,      8,      8,      8,      9,      9,
		10,     10,     10,     11,     11,     12,     13,     13,     14,     14,     15,     16,     17,     17,     18,     19,
		20,     21,     22,     23,     24,     25,     26,     28,     29,     30,     32,     33,     35,     36,     38,     40,
		42,     44,     46,     48,     50,     52,     55,     58,     60,     63,     66,     69,     72,     76,     79,     83,
		87,     91,     96,     100,    105,    110,    115,    120,    126,    132,    138,    145,    151,    159,    166,    174,
		182,    191,    200,    209,    219,    229,    240,    251,    263,    275,    288,    302,    316,    331,    347,    363,
		380,    398,    417,    437,    457,    479,    501,    525,    550,    576,    603,    631,    661,    692,    725,    759,
		794,    832,    871,    912,    955,    1000
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

/*!****************************************************************************
 * @brief
 */
void loadPrm(void){
	auto settingsize = Prm::getSerialSize(Prm::savesys);
	if(!Prm::deserialize(Prm::savesys, &_suser_settings, settingsize)){
		__NOP();
	}
}

/*!****************************************************************************
 * @brief
 */
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
typedef enum {
	BT_NONE,
	BT_LONG_NOW,
	BT_CLICK,
	BT_DOUBLE
}bt_event;

bt_event bt_process(){
	// Settings
	constexpr uint16_t discreteTh = 5;
	constexpr uint16_t longTh = 20;
	constexpr uint16_t nultiClickTh = 30;

	// Button discrete filter
	static uint16_t discreteCnt;
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

	static uint32_t long_cnt;
	if(discreteOut && long_cnt < UINT32_MAX){
		long_cnt++;
	}

	if(discreteOut && long_cnt >= longTh){
		return BT_LONG_NOW;
	}

	static bool discreteOutPrev;
	bool falling = false;
	if(!discreteOut && discreteOutPrev){
		falling = true;
	}
	discreteOutPrev = discreteOut;

	static uint16_t fallingCnt;
	static uint16_t fallingTimer;
	if(falling && long_cnt < longTh){
		fallingCnt++;
		fallingTimer = 0;
	}

	if(fallingTimer < UINT16_MAX){
		fallingTimer++;
	}
	if(fallingTimer == nultiClickTh){
		if(fallingCnt == 1){
			fallingCnt = 0;
			return BT_CLICK;
		}
		if(fallingCnt == 2){
			fallingCnt = 0;
			return BT_DOUBLE;
		}
	}


	if(!discreteOut){
		long_cnt = 0;
	}

	return BT_NONE;
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
	bool outen = Prm::enableout.val ? true : false;
	bool up = btBrightnessIndex == sizeof(brightness2current)/sizeof(brightness2current[0]) - 1 ? false : true;
	TickType_t pxPreviousWakeTime = xTaskGetTickCount();
	while(1){
		bt_event event = bt_process();

		// On/Off
		if(event == BT_CLICK){
			outen = !outen;
			Prm::enableout.val = outen ? 1 : 0;
			savePrm();
		}

		// Brightness
		static bt_event eventPrev;
		if(outen){
			if(eventPrev == BT_LONG_NOW && event != BT_LONG_NOW){
				up = !up;
				savePrm();
			}

			if(event == BT_LONG_NOW){
				if(up){
					if(btBrightnessIndex < sizeof(brightness2current)/sizeof(brightness2current[0]) - 1)
						btBrightnessIndex += 1;
				}
				else{
					if(btBrightnessIndex > 0)
						btBrightnessIndex -= 1;
				}
				Prm::setcurrent.val = brightness2current[btBrightnessIndex];
			}

			if(event == BT_DOUBLE){
				Prm::setcurrent.val = brightness2current[sizeof(brightness2current)/sizeof(brightness2current[0]) - 1];
			}

			LED_OFF();
		}else{
			LED_ON();
		}
		eventPrev = event;

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

		int32_t adcoffset = 24;  /* need calibrate offset */
		// Calc Current
		int32_t current = iqs32_Fy_x1x2y1y2x(adcoffset, 0, Prm::adcCurrent.val, Prm::с_current.val, a.filtered.iled1);
		Prm::current.val = current < 0 ? 0 : current;

		// Calc target in ADC LSB value
		uint16_t targetAdcLsb = iqs32_Fy_x1x2y1y2x(0, 24, Prm::с_current.val, Prm::adcCurrent.val, Prm::setcurrent.val);

		// Calc input voltage
		constexpr int32_t Rh = 100, Rl = 2;
		Prm::input_voltage.val = (a.filtered.vin * vref * (Rh + Rl)) / (4096 * Rl);

		// Termal compensation
		constexpr int16_t termalThreshpoint = 800; // X_X °C
		int16_t termalCompensation = 0;
		if(Prm::temperature.val > termalThreshpoint){
			termalCompensation = (Prm::temperature.val - termalThreshpoint) * 50;
			if(termalCompensation > targetAdcLsb) termalCompensation = targetAdcLsb;
		}
		uint16_t targetcurrent = outen ? targetAdcLsb - termalCompensation : 0;
		a.targetcurrent = /*bypassFilter ?*/ targetcurrent/* : outfilter*/;

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

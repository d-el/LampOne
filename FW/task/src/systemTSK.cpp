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
#include <flash.h>
#include <board.h>
#include <prmSystem.h>
#include <printp.h>
#include <prmSystem.h>
#include "systemTSK.h"
#include "modbusTSK.h"
#include "adcTSK.h"
#include <linearinterp.h>
#include <movingAverageFilter.h>

// MCU calibration data
#define CAL_VREF_DATA	(*(uint16_t*)0x1FFF75AA)	// ADC value VREF_INT at 3.0V VREF
#define CAL_TS_DATA		(*(uint16_t*)0x1FFF75A8)	// ADC value Temperature sensot at 30 °C 3.0V VREF
#define TS_LINEARITY	2.5							// mv / °C

/*!****************************************************************************
* Memory
*/
extern uint8_t _suser_settings;

static const uint16_t brightness2current[] = {
	10,		10,		10,		10,		10,		10,		10,		10,
	11,		11,		11,		11,		11,		11,		11,		11,
	11,		11,		11,		12,		12,		12,		12,		12,
	12,		12,		12,		13,		13,		13,		13,		13,
	14,		14,		14,		14,		14,		15,		15,		15,
	16,		16,		16,		17,		17,		17,		18,		18,
	19,		19,		19,		20,		20,		21,		22,		22,
	23,		23,		24,		25,		26,		26,		27,		28,
	29,		30,		31,		32,		33,		34,		35,		36,
	38,		39,		40,		42,		44,		45,		47,		49,
	50,		52,		54,		57,		59,		61,		64,		66,
	69,		72,		75,		78,		81,		84,		88,		92,
	96,		100,	104,	108,	113,	118,	123,	129,
	134,	140,	146,	153,	159,	167,	174,	182,
	190,	198,	207,	217,	226,	237,	247,	259,
	270,	283,	295,	309,	323,	338,	353,	369,
	386,	404,	423,	442,	463,	484,	506,	530,
	554,	580,	607,	635,	665,	695,	728,	761,
	797,	834,	873,	913,	956,	1000
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
void saveParameter(Prm::Val<uint16_t>& prm, bool read, void *arg){
	(void)arg;
	(void)prm;
	if(read){
		return;
	}
	savePrm();
}


/*!****************************************************************************
 * @brief
 */
void calibrateCurrent(Prm::Val<uint16_t>& prm, bool read, void *arg){
	(void)arg;
	(void)prm;
	if(read){
		return;
	}
	Prm::adcCurrent.val = adcTaskStct.filtered.iled1;
	savePrm();
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

	bt_event eventPrev = BT_NONE;
	bool up = false;
	bool enable = Prm::setcurrent.val != 0;
	static MovingAverageFilter<uint16_t, 64> f_ramp(0);
	bool rampBypass = false;
	uint16_t current = 0;

	TickType_t pxPreviousWakeTime = xTaskGetTickCount();
	while(1){
		uint16_t status = Prm::status & Prm::m_lowInputVoltage;
		bt_event event = bt_process();
		// On/Off
		if(event == BT_CLICK){
			enable = !enable;
			Prm::setcurrent.val = enable ? brightness2current[Prm::powerindex.val] : 0;
			savePrm();
		}

		// Brightness
		if(enable){
			auto maxIndex = sizeof(brightness2current)/sizeof(brightness2current[0]) - 1;
			if(eventPrev == BT_NONE && event == BT_LONG_NOW){
				if(Prm::powerindex.val == 0){
					up = true;
				}else if(Prm::powerindex.val == maxIndex){
					up = false;
				}
				else{
					up = !up;
				}
			}

			if((eventPrev == BT_LONG_NOW || eventPrev == BT_DOUBLE) && event == BT_NONE){
				savePrm();
			}

			if(event == BT_LONG_NOW){
				if(up){
					if(Prm::powerindex.val < maxIndex)
						Prm::powerindex.val += 1;
				}
				else{
					if(Prm::powerindex.val > 0)
						Prm::powerindex.val -= 1;
				}
				Prm::setcurrent.val = brightness2current[Prm::powerindex.val];
				rampBypass = true;
			}

			if(event == BT_DOUBLE){
				if(Prm::powerindex.val == maxIndex){
					Prm::powerindex.val = 0;
				}else{
					Prm::powerindex.val = maxIndex;
				}
				Prm::setcurrent.val = brightness2current[Prm::powerindex.val];
			}
		}
		eventPrev = event;

		Prm::setcurrent ? LED_OFF() : LED_ON();

		// Ramp gen
		uint16_t ramp_current = f_ramp.proc(Prm::setcurrent.val);
		if(ramp_current == Prm::setcurrent.val){
			rampBypass = false;
		}
		if(rampBypass){
			current = Prm::setcurrent.val;
		}else{
			current = ramp_current;
		}

		if(a.filtered.vin == 4095 ||
			a.filtered.iled1 == 4095*8){
			status |= Prm::m_adcOverflow;
		}

		// Calc ADC Vref
		uint16_t vref = ((uint32_t)3000 * CAL_VREF_DATA) / a.filtered.vref;

		// Calc temperature
		int32_t tsCal = (CAL_TS_DATA * a.filtered.vref * 8) / CAL_VREF_DATA;
		int32_t y = tsCal + (4095 * TS_LINEARITY * 100 * 8) / vref;	// ADC value at 130 °C
		Prm::temperature.val = iqs32_Fy_x1x2y1y2x(	tsCal,
												300,						// 30.0 °C
												y,
												1300,						// 130.0 °C
												a.filtered.temperature);

		int32_t adcoffset = a.iled1offset;

		// Calc measure current
		int32_t measCurrent = iqs32_Fy_x1x2y1y2x(adcoffset, 0, Prm::adcCurrent.val, Prm::с_current.val, a.filtered.iled1);
		Prm::current.val = measCurrent < 0 ? 0 : measCurrent;

		// Calc input voltage
		constexpr int32_t Rh = 100, Rl = 2;
		Prm::input_voltage.val = (a.filtered.vin * vref * (Rh + Rl)) / (4096 * Rl);

		// ULVO
		if(Prm::input_voltage.val < Prm::ulvo_voltage.val){
			status |= Prm::m_lowInputVoltage;
		}
		if(Prm::input_voltage.val > Prm::ulvo_voltage.val + Prm::ulvo_hysteresis.val){
			status &= ~Prm::m_lowInputVoltage;
		}

		// Termal compensation
		constexpr uint32_t termalCompensationIq = 1<<12;
		uint32_t termalCompensation = termalCompensationIq;
		if(Prm::temperature.val > Prm::termalThreshpoint.val){
			status |= Prm::m_overheated;
			termalCompensation = (Prm::termalThreshpoint.val * termalCompensationIq) / Prm::temperature.val;
			termalCompensation = (termalCompensation * termalCompensation) / termalCompensationIq;
		}

		// Limit current by input voltage
		if(Prm::input_voltage.val < Prm::voltage_threshold.val){
			int32_t limcurrent = iqs32_Fy_x1x2y1y2x(
					Prm::ulvo_voltage.val, Prm::limit_min_current.val,
					Prm::voltage_threshold.val, Prm::limit_max_current.val,
					Prm::input_voltage.val);
			if(limcurrent < Prm::limit_min_current.val) limcurrent = Prm::limit_min_current.val;
			if(current > limcurrent){
				current = limcurrent;
			}
		}

		// Calc target in ADC LSB value
		uint32_t targetAdcLsb = iqs32_Fy_x1x2y1y2x(0, adcoffset, Prm::с_current.val, Prm::adcCurrent.val, current);
		if(targetAdcLsb <= termalCompensation ||
			status & Prm::m_lowInputVoltage ||
			Prm::setcurrent.val == 0){
			a.targetcurrent = 0;
		}else{
			a.targetcurrent = (targetAdcLsb * termalCompensation) / termalCompensationIq;
		}

		Prm::status.val = status;

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

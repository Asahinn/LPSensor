/*
 * LpTimer.h
 *
 *  Created on: 21 May 2022
 *      Author: Alican
 */

#ifndef LPTIMER_H_
#define LPTIMER_H_


#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define USEC_MULTIP               (1000000U)
#define MSEC_MULTIP               (1000U)
#define WKT_CLK_FREQ         	  8250//(10000U)
#define ACTIVE_PART_IN_DEEPSLEEP  (kPDSLEEPCFG_DeepSleepBODActive | kPDSLEEPCFG_DeepSleepWDTOscActive)
 /*******************************************************************************/
void Init_WakeUpTimer(void);
void Enable_WakeUpTimer(void);
void LowPowerWakeup(void);
void PreEnterLowPower(void);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);




#endif /* LPTIMER_H_ */

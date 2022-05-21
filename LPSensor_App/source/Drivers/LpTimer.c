/*
 * Sensor.c
 *
 *  Created on: 21 May 2022
 *      Author: Alican S.
 */
#include <Drivers/LpTimer.h>
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_common.h"
#include "fsl_power.h"
#include "fsl_usart.h"
#include "fsl_syscon.h"
#include "fsl_wkt.h"
#include "fsl_iocon.h"
#include "fsl_pint.h"
#include "fsl_swm.h"

uint16_t WkCount=0;
void WKT_IRQHandler(void){
    /* Clear interrupt flag.*/
    WKT_ClearStatusFlags(WKT, kWKT_AlarmFlag);
    WKT_StopTimer(WKT);
}
void Init_WakeUpTimer(void){
    wkt_config_t wktConfig;
    /*Activate Low Power Oscillator*/
    	POWER_EnableLPO(true);
    	POWER_EnableLPOInDeepPowerDownMode(true);
    /* Select Low Power Clock Source (10kHz) */
    	wktConfig.clockSource = kWKT_LowPowerClockSource;

    /* Init Wkt */
    	WKT_Init(WKT, &wktConfig);
    /* Clear Pending Interrupt */
    	NVIC_ClearPendingIRQ(WKT_IRQn);
    /* Enable at the NVIC */
    	EnableIRQ(WKT_IRQn);
}
void Enable_WakeUpTimer(void){
    EnableDeepSleepIRQ(WKT_IRQn);
    DisableDeepSleepIRQ(PIN_INT0_IRQn);
}
void LowPowerWakeup(void){
    /* clock configurations restore */
    BOARD_BootClockPll24M();

    CLOCK_EnableClock(kCLOCK_Iocon);
    CLOCK_EnableClock(kCLOCK_Uart0);
}
void PreEnterLowPower(void){
    /* switch main clock source to FRO18M */
    POWER_DisablePD(kPDRUNCFG_PD_FRO_OUT);
    POWER_DisablePD(kPDRUNCFG_PD_FRO);
    CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcFro);
    CLOCK_SetFroOscFreq(kCLOCK_FroOscOut18M);
    CLOCK_SetFroOutClkSrc(kCLOCK_FroSrcFroOsc);

    /* system osc power down
     * application should decide if more part need to power down to achieve better power consumption
     * */
    POWER_EnablePD(kPDRUNCFG_PD_SYSOSC);
    CLOCK_DisableClock(kCLOCK_Iocon);
    CLOCK_DisableClock(kCLOCK_Uart0);
}
void Delay_us(uint32_t us){
	/* Enable Wake up Timer */
		Enable_WakeUpTimer();
		WKT_StartTimer(WKT, USEC_TO_COUNT(us, WKT_CLK_FREQ));
		PreEnterLowPower();
		POWER_EnterPowerDown(ACTIVE_PART_IN_DEEPSLEEP);

        /* restore the active mode configurations */
		LowPowerWakeup();
        WkCount++;
        PRINTF("Wakeup. No: %d\r\n",WkCount);
        WKT_StopTimer(WKT);
}
void Delay_ms(uint32_t ms){
	/* Enable Wake up Timer */
		Enable_WakeUpTimer();
		WKT_StartTimer(WKT, MSEC_TO_COUNT(ms, WKT_CLK_FREQ));
		PreEnterLowPower();
		POWER_EnterPowerDown(ACTIVE_PART_IN_DEEPSLEEP);

        /* restore the active mode configurations */
		LowPowerWakeup();
        WkCount++;
        PRINTF("Wakeup. No: %d\r\n",WkCount);
}
/*******************************************************/

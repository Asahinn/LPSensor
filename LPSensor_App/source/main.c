/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_common.h"
#include "fsl_power.h"
#include "fsl_usart.h"
#include "fsl_syscon.h"
#include "fsl_wkt.h"
#include "fsl_iocon.h"
#include <stdbool.h>
#include "fsl_pint.h"
#include "fsl_swm.h"
#include <Drivers/LpTimer.h>
#include <Drivers/Sensor.h>
/*!
 * @brief Main function
 */
int main(void)
{
	/* Init board hardware. */
		CLOCK_Select(BOARD_DEBUG_USART_CLK_ATTACH);
		BOARD_InitBootPins();
		BOARD_BootClockPll24M();
		BOARD_InitDebugConsole();
		LED_RED_INIT(LOGIC_LED_OFF);
		LED_BLUE_INIT(LOGIC_LED_OFF);
		PRINTF("Low Power Sensor Demo for Thread Motion\r\n");
	/* Init Wake up Timer */
		Init_WakeUpTimer();
		/* start main loop */
		LpSensorApp_Loop();
	    {
			PRINTF("FATAL ERROR!!!\r\n");
			/* force the counter to be placed into memory. */
			static volatile int i = 0;
			i = 0;
			/* enter an infinite loop, just incrementing a counter. */
			while(1) {
				i++ ;
				/* 'dummy' NOP to allow source level single stepping of tight while() loop */
				__asm volatile ("nop");
			}
	    }
}

/*
 * Copyright 2016-2018 NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*******************************************************************************
 * Q-Track 2.cpp
 *
 *  Created on: Jun 20, 2017
 *      Author: Daniel Borisov
 *
 *      This file includes the main loop function for the Q-Track
 *      as well as the  following functions:
 *      - pwrOnSelfTestDAC
 *
 *      This file also contains the following Interrupt Service Routine
 *      Redefined Handlers:
 *      - IntDefaultHandler
 *      - PIT0_IRQHandler
 *      - PIT1_IRQHandler
 *      - PORTA_IRQHandler
 *      - PORTC_IRQHandler
 *      - PORTD_IRQHandler
 *      - UART0_RX_TX_IRQHandler
 *
 *
 *  REVISION HISTORY: (to be filled once code reaches first 'complete' state, or after Dan leaves)
 *      Revisions made:
 *      Date:
 *      Author:
 *
 ******************************************************************************/

/*
 * @file    Q-Track 2.cpp
 * @brief   Application entry point.
 */

/*******************************************************************************
 * Headers
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "board.h"
#include "clock_config.h"
#include "MK50D10.h"
#include "fsl_debug_console.h"
#include "fsl_pit.h"
#include "fsl_uart.h"

#include "parameters.h"
#include "setup.h"

//global variable declared by Q-Track2.h and may be defined in Q-Track2.cpp
#include "Q-Track2.h"
#include "task.h"

/*FUNCTION**********************************************************************
 *
 * Function Name : main
 *
 * 		This is the main loop. It does the following:
 *      - initializes board hardware
 *  	- outputs a beep to indicate beginning of Start up Routines
 * 		- Outputs sine waves on Analog channels to test DAC
 *  	- Sets up CO2 communication and co2 module
 *  	- gets initial RH and temperature reading
 *  	- sets up UART interrupts and PIT
 *  	- infinitely loops a data collection process which
 *  		- over-samples O2 and Pressure (Flow) data while waiting
 *  		  to receive a packet of CO2 data
 *  		- sends data out as analog voltages
 *  		-
 *
 ******************************************************************************/

void my_printf(const char *data)
{
	UART_WriteBlocking(UART4, (const uint8_t *)data, strlen((const char *)data)); //FOR DEBUG
}

/*
 * @brief   Application entry point.
 */

int main(void)
{
    /* Initialize board hardware. */
    BOARD_InitBootClocks();
    gpioSetup();
    uartSetup();
    timerSetup();

    //最好用SysTick作系统调度，因为PIT0这些高级定时器一般有特殊的功能，这里SysTick没有配置对，所以暂用PIT0
    //	SysTick_Config(CLOCK_GetFreq(kCLOCK_CoreSysClk) / 1000U);

    my_printf("run init()\n");

    while(1)
    {
        // 在主循环中查询任务是否准备就绪，调用准备就绪的任务
        // 除此之处主循环中没有其他的代码
        TaskSchedule();
    }
    return 0 ;
}
/*-----------------END MAIN-------------------------------------------------------*/

/* 节拍定时器中断 */
void SysTick_Handler(void)
{

}

/* 在定时器中断中调用TaskPoll函数轮询每个任务是否准备就绪 */
void PIT0_IRQHandler (void)
{
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);// a new interrupt can be generated only after
    TaskPoll();
}

/*ISR for piezoBuzz delay Timer*/
void PIT1_IRQHandler (void)
{
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_1, kPIT_TimerFlag);//To be prepared for the next interrupt trigger event
}

/*ISR for time-stamp of SD card file*/
void PIT2_IRQHandler (void)
{
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_2, kPIT_TimerFlag);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART0_RX_TX_IRQHandler
 * Description   : ISR for sending commands to and receiving data packets from the
 * 					Treymed CO2 Module using UART 0
 *
 * 				- Treymed Module is in Automatic Mode (38400 baud)
 * 				- CO2 Packets come at 60Hz
 * 				- Packets are 12 bytes: '<Wxxxxyyyyz>' or '<wxxxxyyyyz>'
 * 				- Where bytes 11 and 10 are '<W' or '<w' and indicate start of data packet
 * 				- 'xxxx' is CO2 partial pressure data in mmHg
 * 				- 'yyyy' is an ascii hex value from 0000-FFFF for barometric pressure data in mmHg
 * 				- 'z' (byte 11) is the check sum evaluation and is unused in this program
 * 				- '>' indicates the end of a CO2 packet
 * 				- Packet count goes from 9 = 'x' down to 0 = '>'
 * 				- rxIndex goes from 12 down to zero
 *
 ******************************************************************************/
void UART0_RX_TX_IRQHandler(void)
{
	/*If CO2 data packet is being received*/
	if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(UART0))
	{


	}//END IF
}
/*END UART0_RX_TX_IRQHandler-----------------------------------------------------*/

/*FUNCTION**********************************************************************
 *
 * Function Name : UART4_RX_TX_IRQHandler
 * Description   : ISR for sending commands to and receiving data packets from the USB
 *
 * 				- USB Serial Port set to (38400 baud)
 * 				- CO2 Packets come at 60Hz
 * 				- Packets are xx bytes
 * 				- Where bytes xx and xx indicate start of data packet
 * 				- Packet count goes from xx down to xx
 * 				- rxIndex goes from xx down to zero
 *
 ******************************************************************************/
void UART4_RX_TX_IRQHandler(void)
{  //TODO
	/*If USB data packet is being received*/
	if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(UART4))
	{


	}//END if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(UART4))
}
/*END UART0_RX_TX_IRQHandler-----------------------------------------------------*/

void UART1_RX_TX_IRQHandler(void)
{
//	uint8_t data = 0;

	/*receive UART data*/
	if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(UART1))
	{

//		data = UART_ReadByte(UART1);

//		Uart1_Data_Process(data);
	}
}



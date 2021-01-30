/*******************************************************************************
 * setup.h
 *
 *  Created on: June 20, 2017
 *      Author: Daniel Borisov
 *
 *      This header file declares variables and functions used in setup.cpp
 *
 *  REVISION HISTORY: (to be filled once code reaches first 'complete' state, or after Dan leaves)
 *      Revisions made:
 *      Date:
 *      Author:
 ******************************************************************************/

#ifndef SOURCE_SETUP_H_
#define SOURCE_SETUP_H_

#include "parameters.h"
#include "fsl_adc16.h"
#include "fsl_uart.h"

extern adc16_channel_config_t adcOxygenConfig, adcPresConfig, adcPowerConfig, adcGndConfig, adcExtraConfig, adcExtraDiffConfig;
extern uart_handle_t g_uartHandle;

void setup();
void adcSetup();
void gpioSetup();
void uartSetup();
void uartInterruptSetup();
void usbSetup();
void spiSetup();
void i2cSetup();
void timerSetup();
bool btPair();

#endif /* SOURCE_SETUP_H_ */

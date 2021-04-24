/*******************************************************************************
 * setup.cpp
 *
 *  Created on: Jun 20, 2017
 *      Author: Daniel Borisov
 *
 *      This file contains set-up functions for MCU peripherals
 *      and on-board modules:
 *      - Set-up for general input/output pins
 *      - ADC Set-up
 *      - UART User Callback
 *      - UART Set-up
 *      - UART interrupt Setup
 *      - I2C Set-up
 *      - SPI Set-up
 *      - Timer (Periodic Interrupt Timer) Set up
 *      - TODO: pairing to BlueTooth
 *      - CO2 Board Set-Up
 *
 *  REVISION HISTORY: (to be filled once code reaches first 'complete' state, or after Dan leaves)
 *      Revisions made:
 *      Date:
 *      Author:
 ******************************************************************************/


/*******************************************************************************
 * Header Files
 ******************************************************************************/
#include "setup.h"
#include "fsl_common.h"
#include "fsl_i2c.h"
#include "fsl_dspi.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_sim.h"
#include "fsl_pit.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "Q-Track2.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*16 bit ADC variables*/
adc16_channel_config_t adcOxygenConfig, adcPresConfig, adcPowerConfig, adcGndConfig, adcExtraConfig, adcExtraDiffConfig;

adc16_config_t adcUserConfig;
adc16_hardware_compare_config_t adcUserHardwareConfig;

/*UART variables for CO2 and USB Comm*/
uart_config_t uartCarbonConfig, uartUsbConfig, uartBtConfig;
uart_handle_t g_uartHandle;
uint8_t g_rxRingBuffer[USB_BUFFER_SIZE] = {0}; /* RX ring buffer. */

/*i2c variables for RH and LCD*/
i2c_master_config_t rhCommConfig, lcdCommConfig;

/*Port Pin and GPIO Variables*/
port_pin_config_t unusedPortsConfig;
PORT_Type *portsArray[] = {PORTA, PORTB, PORTC, PORTD, PORTE};
GPIO_Type *gpioArray[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE};


/*FUNCTION**********************************************************************
 *
 * Function Name : gpioSetup
 * Inputs: None
 * Outputs: None
 * Description   : This function initializes general input/output pins
 *
 *END**************************************************************************/
void gpioSetup(){

	/*Set the System Clock Gating Control Register*/
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

	/*Set up BlueTooth Reset Pin*/
#ifdef A_BT_RESETA
	gpio_pin_config_t btResetPinConfig;
	port_pin_config_t btResetPortConfig;
	btResetPortConfig.mux = kPORT_MuxAsGpio;
	btResetPortConfig.pullSelect = kPORT_PullUp;
	btResetPortConfig.slewRate = kPORT_SlowSlewRate;
	btResetPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	btResetPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	btResetPortConfig.driveStrength = kPORT_LowDriveStrength;
	btResetPortConfig.lockRegister = kPORT_LockRegister;
#endif

	/*Set up BlueTooth Pairing Button Pin*/
#ifdef A_BT_PAIR_BUTTON
	gpio_pin_config_t btPairBtnPinConfig;
	port_pin_config_t btPairBtnPortConfig;

	btPairBtnPortConfig.mux = A_BT_PAIR_BUTTON_MUX;
	btPairBtnPortConfig.pullSelect = kPORT_PullUp;
	btPairBtnPortConfig.slewRate = kPORT_FastSlewRate;
	btPairBtnPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	btPairBtnPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	btPairBtnPortConfig.driveStrength = kPORT_LowDriveStrength;
	btPairBtnPortConfig.lockRegister = kPORT_UnlockRegister;

	btPairBtnPinConfig.pinDirection = kGPIO_DigitalInput;
	btPairBtnPinConfig.outputLogic = 0;

	PORT_SetPinConfig(PORTA, 12, &btPairBtnPortConfig);
	PORT_SetPinInterruptConfig(PORTA, 12, kPORT_InterruptFallingEdge);
	GPIO_PinInit(GPIOA, 12, &btPairBtnPinConfig);
#endif

	/*Set up Start/Stop SD Data Collection Button Pin **Not yet implemented** */
#ifdef A_START_STOP_BUTTON
	gpio_pin_config_t startStopButnPinConfig;
	port_pin_config_t startStopButnPortConfig;

	startStopButnPortConfig.mux = A_START_STOP_BUTTON_MUX;
	startStopButnPortConfig.pullSelect = kPORT_PullUp;
	startStopButnPortConfig.slewRate = kPORT_FastSlewRate;
	startStopButnPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	startStopButnPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	startStopButnPortConfig.driveStrength = kPORT_LowDriveStrength;
	startStopButnPortConfig.lockRegister = kPORT_UnlockRegister;

	startStopButnPinConfig.pinDirection = kGPIO_DigitalInput;
	startStopButnPinConfig.outputLogic = 0;

	PORT_SetPinConfig(PORTA, 13, &startStopButnPortConfig);
//	PORT_SetPinInterruptConfig(PORTA, 13, kPORT_InterruptFallingEdge);
	GPIO_PinInit(GPIOA, 13, &startStopButnPinConfig);
#endif

	/*Set up Pressure Zero (Flow Zero) Button Pin*/
#ifdef A_PRES_ZERO_BUTTON
	gpio_pin_config_t presZeroButnPinConfig;
	port_pin_config_t presZeroButnPortConfig;

	presZeroButnPortConfig.mux = A_PRES_ZERO_BUTTON_MUX;
	presZeroButnPortConfig.pullSelect = kPORT_PullUp;
	presZeroButnPortConfig.slewRate = kPORT_FastSlewRate;
	presZeroButnPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	presZeroButnPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	presZeroButnPortConfig.driveStrength = kPORT_LowDriveStrength;
	presZeroButnPortConfig.lockRegister = kPORT_UnlockRegister;

	presZeroButnPinConfig.pinDirection = kGPIO_DigitalInput;
	presZeroButnPinConfig.outputLogic = 0;

	PORT_SetPinConfig(PORTA, 14, &presZeroButnPortConfig);
	PORT_SetPinInterruptConfig(PORTA, 14, kPORT_InterruptFallingEdge);
	GPIO_PinInit(GPIOA, 14, &presZeroButnPinConfig);
#endif

	/*Set up Factory Calibration Header Pin*/ //TODO ? why A_CAL_HEADER is not defined?
#ifdef A_CAL_HEADER
	gpio_pin_config_t calHeaderPinConfig;
	port_pin_config_t calHeaderPortConfig;

	calHeaderPortConfig.mux = A_CAL_HEADER_MUX;
	calHeaderPortConfig.pullSelect = kPORT_PullDisable;
	calHeaderPortConfig.slewRate = kPORT_FastSlewRate;
	calHeaderPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	calHeaderPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	calHeaderPortConfig.driveStrength = kPORT_LowDriveStrength;
	calHeaderPortConfig.lockRegister = kPORT_UnlockRegister;

	calHeaderPinConfig.pinDirection = kGPIO_DigitalInput;
	calHeaderPinConfig.outputLogic = 0;

	PORT_SetPinConfig(PORTA, 15, &calHeaderPortConfig);
	PORT_SetPinInterruptConfig(PORTA, 15, kPORT_InterruptOrDMADisabled);
	GPIO_PinInit(GPIOA, 15, &calHeaderPinConfig);
#endif

	/*Unused*/
#ifdef A_PTA18
	//gpio_pin_config_t;
	//port_pin_config_t;
#endif

	/*Unused*/
#ifdef A_PTA19
	//gpio_pin_config_t;
	//port_pin_config_t;
#endif

	/*Set up ADC Pressure Signal Pin*/ //for flow
#ifdef B_ADC0_SE19_PRES_SIG//should be SE9, no SE19
	port_pin_config_t presAdcPortConfig;

	presAdcPortConfig.mux = B_ADC0_SE19_PRES_SIG_MUX;
	presAdcPortConfig.pullSelect = kPORT_PullDisable;
	presAdcPortConfig.slewRate = kPORT_FastSlewRate;
	presAdcPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	presAdcPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	presAdcPortConfig.driveStrength = kPORT_HighDriveStrength;
	presAdcPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTB, 1, &presAdcPortConfig);
	PORT_SetPinInterruptConfig(PORTB, 1, kPORT_InterruptOrDMADisabled);
#endif

	/*Set up I2C Clock Signal Pin for LCD*/
#ifdef B_I2C0_SCL_LCD
	port_pin_config_t lcdClkPortConfig;

	lcdClkPortConfig.mux = B_I2C0_SCL_DIG_POT_MUX;
	lcdClkPortConfig.pullSelect = kPORT_PullDisable;
	lcdClkPortConfig.slewRate = kPORT_FastSlewRate;
	lcdClkPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	lcdClkPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	lcdClkPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTB, 2, &lcdClkPortConfig);
	PORT_SetPinInterruptConfig(PORTB, 2, kPORT_InterruptOrDMADisabled);
#endif

	/*Set up I2C Data Signal Pin for the Digital Pot*/
#ifdef B_I2C0_SDA_LCD
	port_pin_config_t lcdDatPortConfig;

	lcdDatPortConfig.mux = B_I2C0_SDA_DIG_POT_MUX;
	lcdDatPortConfig.pullSelect = kPORT_PullDisable;
	lcdDatPortConfig.slewRate = kPORT_FastSlewRate;
	lcdDatPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	lcdDatPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	lcdDatPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTB, 3, &lcdDatPortConfig);
	PORT_SetPinInterruptConfig(PORTB, 3, kPORT_InterruptOrDMADisabled);
#endif

	/*Set up UART3 receive Pin (BT)*/
#ifdef B_UART1_RX
		port_pin_config_t buletoothUartRxPortConfig;

		buletoothUartRxPortConfig.mux = B_UART1_RX_MUX;
		buletoothUartRxPortConfig.pullSelect = kPORT_PullDisable;
		buletoothUartRxPortConfig.slewRate = kPORT_FastSlewRate;
		buletoothUartRxPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
		buletoothUartRxPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
		buletoothUartRxPortConfig.lockRegister = kPORT_UnlockRegister;

		PORT_SetPinConfig(PORTC, 3, &buletoothUartRxPortConfig);
		PORT_SetPinInterruptConfig(PORTC, 3, kPORT_InterruptOrDMADisabled);
#endif

		/*Set up UART3 Transmission Pin (BT)*/
#ifdef B_UART1_TX
		port_pin_config_t buletoothUartTxPortConfig;

		buletoothUartTxPortConfig.mux = B_UART1_TX_MUX;
		buletoothUartTxPortConfig.pullSelect = kPORT_PullDisable;
		buletoothUartTxPortConfig.slewRate = kPORT_FastSlewRate;
		buletoothUartTxPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
		buletoothUartTxPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
		buletoothUartTxPortConfig.lockRegister = kPORT_UnlockRegister;

		PORT_SetPinConfig(PORTC, 4, &buletoothUartTxPortConfig);
		PORT_SetPinInterruptConfig(PORTC, 4, kPORT_InterruptOrDMADisabled);
#endif

	/*Set up UART0 Receiver Pin (Receives Transmission from CO2)*/
#ifdef B_UART0_RX_CO2TX
	port_pin_config_t carbonUartRxPortConfig;

	carbonUartRxPortConfig.mux = B_UART0_RX_CO2TX_MUX;
	carbonUartRxPortConfig.pullSelect = kPORT_PullDisable;
	carbonUartRxPortConfig.slewRate = kPORT_FastSlewRate;
	carbonUartRxPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	carbonUartRxPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	carbonUartRxPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTB, 16, &carbonUartRxPortConfig);
	PORT_SetPinInterruptConfig(PORTB, 16, kPORT_InterruptOrDMADisabled);
#endif

	/*Set up UART0 Transmission Pin (CO2 Receives)*/
#ifdef B_UART0_TX_CO2RX
	port_pin_config_t carbonUartTxPortConfig;

	carbonUartTxPortConfig.mux = B_UART0_TX_CO2RX_MUX;
	carbonUartTxPortConfig.pullSelect = kPORT_PullDisable;
	carbonUartTxPortConfig.slewRate = kPORT_FastSlewRate;
	carbonUartTxPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	carbonUartTxPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	carbonUartTxPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTB, 17, &carbonUartTxPortConfig);
	PORT_SetPinInterruptConfig(PORTB, 17, kPORT_InterruptOrDMADisabled);
#endif

	/*Set up Green LED Pin*/
#ifdef B_GREEN_LED
	gpio_pin_config_t greenLedPinConfig;
	port_pin_config_t greenLedPortConfig;

	greenLedPortConfig.mux = B_GREEN_LED_MUX;
	greenLedPortConfig.pullSelect = kPORT_PullDisable;
	greenLedPortConfig.slewRate = kPORT_FastSlewRate;
	greenLedPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	greenLedPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	greenLedPortConfig.driveStrength = kPORT_HighDriveStrength;
	greenLedPortConfig.lockRegister = kPORT_UnlockRegister;

	greenLedPinConfig.pinDirection = kGPIO_DigitalOutput;
	greenLedPinConfig.outputLogic = 0;

	PORT_SetPinConfig(PORTB, 18, &greenLedPortConfig);
	PORT_SetPinInterruptConfig(PORTB, 18, kPORT_InterruptOrDMADisabled);
	GPIO_PinInit(GPIOB, 18, &greenLedPinConfig);
#endif

	/*Set up Red LED Pin*/
#ifdef B_RED_LED
	gpio_pin_config_t redLedPinConfig;
	port_pin_config_t redLedPortConfig;

	redLedPortConfig.mux = B_RED_LED_MUX;
	redLedPortConfig.pullSelect = kPORT_PullDisable;
	redLedPortConfig.slewRate = kPORT_FastSlewRate;
	redLedPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	redLedPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	redLedPortConfig.driveStrength = kPORT_HighDriveStrength;
	redLedPortConfig.lockRegister = kPORT_UnlockRegister;

	redLedPinConfig.pinDirection = kGPIO_DigitalOutput;
	redLedPinConfig.outputLogic = 0;

	PORT_SetPinConfig(PORTB, 19, &redLedPortConfig);
	PORT_SetPinInterruptConfig(PORTB, 19, kPORT_InterruptOrDMADisabled);
	GPIO_PinInit(GPIOB, 19, &redLedPinConfig);
#endif

	/*Set up SPI Peripheral Chip Select Pin0 for DAC*/
#ifdef B_SPI2_PCS0_DAC
	port_pin_config_t dacCsPortConfig;
	dacCsPortConfig.mux = B_SPI2_PCS0_DAC_MUX;
	dacCsPortConfig.pullSelect = kPORT_PullDisable;
	dacCsPortConfig.slewRate = kPORT_FastSlewRate;
	dacCsPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	dacCsPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	dacCsPortConfig.driveStrength = kPORT_HighDriveStrength;
	dacCsPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTB, 20, &dacCsPortConfig);
	PORT_SetPinInterruptConfig(PORTB, 20, kPORT_InterruptOrDMADisabled);

#endif

	/*Set up SPI Clock Signal Pin for DAC*/
#ifdef B_SPI2_SCK_DAC
	port_pin_config_t dacSckPortConfig;
	dacSckPortConfig.mux = B_SPI2_SCK_DAC_MUX;
	dacSckPortConfig.pullSelect = kPORT_PullDisable;
	dacSckPortConfig.slewRate = kPORT_FastSlewRate;
	dacSckPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	dacSckPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	dacSckPortConfig.driveStrength = kPORT_HighDriveStrength;
	dacSckPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTB, 21, &dacSckPortConfig);
	PORT_SetPinInterruptConfig(PORTB, 21, kPORT_InterruptOrDMADisabled);
#endif

	/*Set up SPI Signal Out pin from DAC*/
#ifdef B_SPI2_SOUT_DAC
	port_pin_config_t dacOutPortConfig;
	dacOutPortConfig.mux = B_SPI2_SOUT_DAC_MUX;
	dacOutPortConfig.pullSelect = kPORT_PullDisable;
	dacOutPortConfig.slewRate = kPORT_FastSlewRate;
	dacOutPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	dacOutPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	dacOutPortConfig.driveStrength = kPORT_HighDriveStrength;
	dacOutPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTB, 22, &dacOutPortConfig);
	PORT_SetPinInterruptConfig(PORTB, 22, kPORT_InterruptOrDMADisabled);
#endif

	/*Set up ADC O2 Signal Pin*/
#ifdef C_ADC0_SE14_O2_SIG
	port_pin_config_t oxyAdcPortConfig;

	oxyAdcPortConfig.mux = C_ADC0_SE14_O2_SIG_MUX;
	oxyAdcPortConfig.pullSelect = kPORT_PullDisable;
	oxyAdcPortConfig.slewRate = kPORT_FastSlewRate;
	oxyAdcPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	oxyAdcPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	oxyAdcPortConfig.driveStrength = kPORT_HighDriveStrength;
	oxyAdcPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTC, 0, &oxyAdcPortConfig);
	PORT_SetPinInterruptConfig(PORTC, 0, kPORT_InterruptOrDMADisabled);
#endif

	/*Set up I2C Clock Signal Pin for RH and Temp*/
#ifdef C_I2C1_SCL_RH_CLK
	port_pin_config_t tempHumidClkPortConfig;

	tempHumidClkPortConfig.mux = C_I2C1_SCL_RH_CLK_MUX;
	tempHumidClkPortConfig.pullSelect = kPORT_PullDisable; //If either pull -up or -down enalbed, the pin is input
	tempHumidClkPortConfig.slewRate = kPORT_FastSlewRate;
	tempHumidClkPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	tempHumidClkPortConfig.openDrainEnable = kPORT_OpenDrainEnable;//must be pulled-up externally for this setting
	                                                               //the pin is output
	tempHumidClkPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTC, 10, &tempHumidClkPortConfig);
	PORT_SetPinInterruptConfig(PORTC, 10, kPORT_InterruptOrDMADisabled);
#endif

	/*Set up I2C Data Signal Pin for RH and Temp*/
#ifdef C_I2C1_SDA_RH_DAT
	port_pin_config_t tempHumidDatPortConfig;

	tempHumidDatPortConfig.mux = C_I2C1_SDA_RH_DAT_MUX;
	tempHumidDatPortConfig.pullSelect = kPORT_PullDisable;
	tempHumidDatPortConfig.slewRate = kPORT_FastSlewRate;
	tempHumidDatPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	tempHumidDatPortConfig.openDrainEnable = kPORT_OpenDrainEnable;
	tempHumidDatPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTC, 11, &tempHumidDatPortConfig);
	PORT_SetPinInterruptConfig(PORTC, 11, kPORT_InterruptOrDMADisabled);
#endif

	/*Set up UART4 Request to send to USB Pin (USB Clears to receive from UART)*/
#ifdef C_UART4_RTS_USB_CTS
	port_pin_config_t usbUartRtsPortConfig;

	usbUartRtsPortConfig.mux = C_UART4_RTS_USB_CTS_MUX;
	usbUartRtsPortConfig.pullSelect = kPORT_PullDisable;
	usbUartRtsPortConfig.slewRate = kPORT_FastSlewRate;
	usbUartRtsPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	usbUartRtsPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	usbUartRtsPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTC, 12, &usbUartRtsPortConfig);
	PORT_SetPinInterruptConfig(PORTC, 12, kPORT_InterruptOrDMADisabled);
#endif

	/*Set up UART4 Clear to receive from USB Pin (USB Requests to Send to UART)*/
#ifdef C_UART4_CTS_USB_RTS
	port_pin_config_t usbUartCtsPortConfig;

	usbUartCtsPortConfig.mux = C_UART4_CTS_USB_RTS_MUX;
	usbUartCtsPortConfig.pullSelect = kPORT_PullDisable;
	usbUartCtsPortConfig.slewRate = kPORT_FastSlewRate;
	usbUartCtsPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	usbUartCtsPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	usbUartCtsPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTC, 13, &usbUartCtsPortConfig);
	PORT_SetPinInterruptConfig(PORTC, 13, kPORT_InterruptOrDMADisabled);
#endif

	/*Set up UART4 Receive Pin (USB Transmits to UART)*/
#ifdef C_UART4_RX_USB_TX
	port_pin_config_t usbUartRxPortConfig;

	usbUartRxPortConfig.mux = C_UART4_RX_USB_TX_MUX;
	usbUartRxPortConfig.pullSelect = kPORT_PullDisable;
	usbUartRxPortConfig.slewRate = kPORT_FastSlewRate;
	usbUartRxPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	usbUartRxPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	usbUartRxPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTC, 14, &usbUartRxPortConfig);
	PORT_SetPinInterruptConfig(PORTC, 14, kPORT_InterruptOrDMADisabled);
#endif

	/*Set up UART4 Transmission Pin (USB Receives from UART)*/
#ifdef C_UART4_TX_USB_RX
	port_pin_config_t usbUartTxPortConfig;

	usbUartTxPortConfig.mux = C_UART4_TX_USB_RX_MUX;
	usbUartTxPortConfig.pullSelect = kPORT_PullDisable;
	usbUartTxPortConfig.slewRate = kPORT_FastSlewRate;
	usbUartTxPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	usbUartTxPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	usbUartTxPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTC, 15, &usbUartTxPortConfig);
	PORT_SetPinInterruptConfig(PORTC, 15, kPORT_InterruptOrDMADisabled);
#endif

	/*Set up CO2 Zero Button Pin*/
#ifdef C_ZERO_CO2_BUTTON
	gpio_pin_config_t zeroButnPinConfig;
	port_pin_config_t zeroButnPortConfig;

	zeroButnPortConfig.mux = C_ZERO_CO2_BUTTON_MUX;
	zeroButnPortConfig.pullSelect = kPORT_PullUp;
	zeroButnPortConfig.slewRate = kPORT_FastSlewRate;
	zeroButnPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	zeroButnPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	zeroButnPortConfig.driveStrength = kPORT_LowDriveStrength;
	zeroButnPortConfig.lockRegister = kPORT_UnlockRegister;

	zeroButnPinConfig.pinDirection = kGPIO_DigitalInput;
	zeroButnPinConfig.outputLogic = 0;

	PORT_SetPinConfig(PORTC, 16, &zeroButnPortConfig);
	PORT_SetPinInterruptConfig(PORTC, 16, kPORT_InterruptFallingEdge);
	GPIO_PinInit(GPIOC, 16, &zeroButnPinConfig);
#endif

	/*Set up Pressure (Flow) Zero Button Pin*/ //not used for now, why this button
#ifdef C_PRES_ZERO_CS
	gpio_pin_config_t presZeroCSPinConfig;
	port_pin_config_t presZeroCSPortConfig;

	presZeroCSPortConfig.mux = C_PRES_ZERO_CS_MUX;
	presZeroCSPortConfig.pullSelect = kPORT_PullDisable;
	presZeroCSPortConfig.slewRate = kPORT_FastSlewRate;
	presZeroCSPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	presZeroCSPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	presZeroCSPortConfig.driveStrength = kPORT_HighDriveStrength;
	presZeroCSPortConfig.lockRegister = kPORT_UnlockRegister;

	presZeroCSPinConfig.pinDirection = kGPIO_DigitalOutput;
	presZeroCSPinConfig.outputLogic = 1;

	PORT_SetPinConfig(PORTC, 17, &presZeroCSPortConfig);
	PORT_SetPinInterruptConfig(PORTC, 17, kPORT_InterruptOrDMADisabled);
	GPIO_PinInit(GPIOC, 17, &presZeroCSPinConfig);
#endif

	/*Set up Factory Calibration Mode TODO: Incremental Control??? Pin*/
#ifdef C_INC_CTRL
	gpio_pin_config_t incCtrlPinConfig;
	port_pin_config_t incCtrlPortConfig;

	incCtrlPortConfig.mux = C_INC_CTRL_MUX;
	incCtrlPortConfig.pullSelect = kPORT_PullDisable;
	incCtrlPortConfig.slewRate = kPORT_FastSlewRate;
	incCtrlPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	incCtrlPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	incCtrlPortConfig.driveStrength = kPORT_HighDriveStrength;
	incCtrlPortConfig.lockRegister = kPORT_UnlockRegister;

	incCtrlPinConfig.pinDirection = kGPIO_DigitalOutput;
	incCtrlPinConfig.outputLogic = 1;

	PORT_SetPinConfig(PORTC, 18, &incCtrlPortConfig);
	PORT_SetPinInterruptConfig(PORTC, 18, kPORT_InterruptOrDMADisabled);
	GPIO_PinInit(GPIOC, 18, &incCtrlPinConfig);
#endif

	/*Set up O2 Span Chip Select Pin*/
#ifdef D_O2_SPAN_CS
	gpio_pin_config_t oxySpanCSPinConfig;
	port_pin_config_t oxySpanCSPortConfig;

	oxySpanCSPortConfig.mux = D_O2_SPAN_CS_MUX;
	oxySpanCSPortConfig.pullSelect = kPORT_PullDisable;
	oxySpanCSPortConfig.slewRate = kPORT_FastSlewRate;
	oxySpanCSPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	oxySpanCSPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	oxySpanCSPortConfig.driveStrength = kPORT_HighDriveStrength;
	oxySpanCSPortConfig.lockRegister = kPORT_UnlockRegister;

	oxySpanCSPinConfig.pinDirection = kGPIO_DigitalOutput;
	oxySpanCSPinConfig.outputLogic = 1;

	PORT_SetPinConfig(PORTD, 2, &oxySpanCSPortConfig);
	PORT_SetPinInterruptConfig(PORTD, 2, kPORT_InterruptOrDMADisabled);
	GPIO_PinInit(GPIOD, 2, &oxySpanCSPinConfig);
#endif

	/*TODO Set digital Pot control pin?*/
#ifdef D_UD_POT_CTRL
	gpio_pin_config_t udPotCtrlPinConfig;
	port_pin_config_t udPotCtrlPortConfig;

	udPotCtrlPortConfig.mux = D_UD_POT_CTRL_MUX;
	udPotCtrlPortConfig.pullSelect = kPORT_PullDisable;
	udPotCtrlPortConfig.slewRate = kPORT_FastSlewRate;
	udPotCtrlPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	udPotCtrlPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	udPotCtrlPortConfig.driveStrength = kPORT_HighDriveStrength;
	udPotCtrlPortConfig.lockRegister = kPORT_UnlockRegister;

	udPotCtrlPinConfig.pinDirection = kGPIO_DigitalOutput;
	udPotCtrlPinConfig.outputLogic = 1;

	PORT_SetPinConfig(PORTD, 3, &udPotCtrlPortConfig);
	PORT_SetPinInterruptConfig(PORTD, 3, kPORT_InterruptOrDMADisabled);
	GPIO_PinInit(GPIOD, 3, &udPotCtrlPinConfig);
#endif

	/*Set up O2 Calibration Pin*/
#ifdef D_O2_CAL_BUTTON
	gpio_pin_config_t oxyCalButnPinConfig;
	port_pin_config_t oxyCalButnPortConfig;

	oxyCalButnPortConfig.mux = D_O2_CAL_BUTTON_MUX;
	oxyCalButnPortConfig.pullSelect = kPORT_PullUp;
	oxyCalButnPortConfig.slewRate = kPORT_FastSlewRate;
	oxyCalButnPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	oxyCalButnPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	oxyCalButnPortConfig.driveStrength = kPORT_LowDriveStrength;
	oxyCalButnPortConfig.lockRegister = kPORT_UnlockRegister;

	oxyCalButnPinConfig.pinDirection = kGPIO_DigitalInput;
	oxyCalButnPinConfig.outputLogic = 0;

	PORT_SetPinConfig(PORTD, 4, &oxyCalButnPortConfig);
	PORT_SetPinInterruptConfig(PORTD, 4, kPORT_InterruptFallingEdge);//IRQC=1010, falling edge

	GPIO_PinInit(GPIOD, 4, &oxyCalButnPinConfig);
#endif

	/*Set up Piezo Speaker Control Pin*/
#ifdef D_PIEZO_CTRL
	gpio_pin_config_t piezoCtrlPinConfig;
	port_pin_config_t piezoCtrlPortConfig;

	piezoCtrlPortConfig.mux = D_PIEZO_CTRL_MUX;
	piezoCtrlPortConfig.pullSelect = kPORT_PullDisable;
	piezoCtrlPortConfig.slewRate = kPORT_FastSlewRate;
	piezoCtrlPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	piezoCtrlPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	piezoCtrlPortConfig.driveStrength = kPORT_HighDriveStrength;
	piezoCtrlPortConfig.lockRegister = kPORT_UnlockRegister;

	piezoCtrlPinConfig.pinDirection = kGPIO_DigitalOutput;
	piezoCtrlPinConfig.outputLogic = 0;

	PORT_SetPinConfig(PORTD, 5, &piezoCtrlPortConfig);
	PORT_SetPinInterruptConfig(PORTD, 5, kPORT_InterruptOrDMADisabled);
	GPIO_PinInit(GPIOD, 5, &piezoCtrlPinConfig);
#endif

	/*Set up Calibration Header Pin*/
#ifdef D_CAL_HEADER_D
	gpio_pin_config_t calHeaderPinConfig;
	port_pin_config_t calHeaderPortConfig;

	calHeaderPortConfig.mux = D_CAL_HEADER_MUX;
	calHeaderPortConfig.pullSelect = kPORT_PullDisable;
	calHeaderPortConfig.slewRate = kPORT_FastSlewRate;
	calHeaderPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	calHeaderPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	calHeaderPortConfig.driveStrength = kPORT_LowDriveStrength;
	calHeaderPortConfig.lockRegister = kPORT_UnlockRegister;

	calHeaderPinConfig.pinDirection = kGPIO_DigitalInput;
	calHeaderPinConfig.outputLogic = 0;

	PORT_SetPinConfig(PORTD, 6, &calHeaderPortConfig);
	PORT_SetPinInterruptConfig(PORTD, 6, kPORT_InterruptOrDMADisabled);
	GPIO_PinInit(GPIOD, 6, &calHeaderPinConfig);
#endif

//YC_forSD
#ifdef D_SD_DETECT
	gpio_pin_config_t sd0DetectPinConfig;
	port_pin_config_t sd0DetectPortConfig;

	sd0DetectPortConfig.mux = D_SD_DETECT_MUX;
	sd0DetectPortConfig.pullSelect = kPORT_PullUp;//needs to be determined
	sd0DetectPortConfig.slewRate = kPORT_FastSlewRate;
	sd0DetectPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	sd0DetectPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	sd0DetectPortConfig.driveStrength = kPORT_HighDriveStrength;
	sd0DetectPortConfig.lockRegister = kPORT_UnlockRegister;

	sd0DetectPinConfig.pinDirection = kGPIO_DigitalInput;
	sd0DetectPinConfig.outputLogic = 0;

	PORT_SetPinConfig(PORTD, 7, &sd0DetectPortConfig);
	PORT_SetPinInterruptConfig(PORTD, 7, kPORT_InterruptEitherEdge);//interrupt on both falling and rising edges
	GPIO_PinInit(GPIOD, 7, &sd0DetectPinConfig);
#endif

#ifdef E_SDHC0_D1
//	gpio_pin_config_t sd0D1PinConfig;
	port_pin_config_t sd0D1PortConfig;

	sd0D1PortConfig.mux = E_SDHC0_D1_MUX;
	sd0D1PortConfig.pullSelect = kPORT_PullUp;
	sd0D1PortConfig.slewRate = kPORT_FastSlewRate;
	sd0D1PortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	sd0D1PortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	sd0D1PortConfig.driveStrength = kPORT_HighDriveStrength;
	sd0D1PortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTE, 0, &sd0D1PortConfig);
	PORT_SetPinInterruptConfig(PORTE, 0, kPORT_InterruptOrDMADisabled);
#endif

#ifdef E_SDHC0_D0
	port_pin_config_t sd0D0PortConfig;

	sd0D0PortConfig.mux = E_SDHC0_D0_MUX;
	sd0D0PortConfig.pullSelect = kPORT_PullUp;
	sd0D0PortConfig.slewRate = kPORT_FastSlewRate;
	sd0D0PortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	sd0D0PortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	sd0D0PortConfig.driveStrength = kPORT_HighDriveStrength;
	sd0D0PortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTE, 1, &sd0D0PortConfig);
	PORT_SetPinInterruptConfig(PORTE, 1, kPORT_InterruptOrDMADisabled);
#endif

#ifdef E_SDHC0_DCLK
	port_pin_config_t sd0ClkPortConfig;

	sd0ClkPortConfig.mux = E_SDHC0_DCLK_MUX;
	sd0ClkPortConfig.pullSelect = kPORT_PullDisable;
	sd0ClkPortConfig.slewRate = kPORT_FastSlewRate;
	sd0ClkPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	sd0ClkPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	sd0ClkPortConfig.driveStrength = kPORT_HighDriveStrength;
	sd0ClkPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTE, 2, &sd0ClkPortConfig);
	PORT_SetPinInterruptConfig(PORTE, 2, kPORT_InterruptOrDMADisabled);
#endif

#ifdef E_SDHC0_CMD
	port_pin_config_t sd0CmdPortConfig;

	sd0CmdPortConfig.mux = E_SDHC0_CMD_MUX;
	sd0CmdPortConfig.pullSelect = kPORT_PullUp;
	sd0CmdPortConfig.slewRate = kPORT_FastSlewRate;
	sd0CmdPortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	sd0CmdPortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	sd0CmdPortConfig.driveStrength = kPORT_HighDriveStrength;
	sd0CmdPortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTE, 3, &sd0CmdPortConfig);
	PORT_SetPinInterruptConfig(PORTE, 3, kPORT_InterruptOrDMADisabled);
#endif

#ifdef E_SDHC0_D3
	port_pin_config_t sd0D3PortConfig;

	sd0D3PortConfig.mux = E_SDHC0_D3_MUX;
	sd0D3PortConfig.pullSelect = kPORT_PullUp;
	sd0D3PortConfig.slewRate = kPORT_FastSlewRate;
	sd0D3PortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	sd0D3PortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	sd0D3PortConfig.driveStrength = kPORT_HighDriveStrength;
	sd0D3PortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTE, 4, &sd0D3PortConfig);
	PORT_SetPinInterruptConfig(PORTE, 4, kPORT_InterruptOrDMADisabled);
#endif

#ifdef E_SDHC0_D2
	port_pin_config_t sd0D2PortConfig;

	sd0D2PortConfig.mux = E_SDHC0_D2_MUX;
	sd0D2PortConfig.pullSelect = kPORT_PullDisable;
	sd0D2PortConfig.slewRate = kPORT_FastSlewRate;
	sd0D2PortConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
	sd0D2PortConfig.openDrainEnable = kPORT_OpenDrainDisable;
	sd0D2PortConfig.driveStrength = kPORT_HighDriveStrength;
	sd0D2PortConfig.lockRegister = kPORT_UnlockRegister;

	PORT_SetPinConfig(PORTE, 5, &sd0D2PortConfig);
	PORT_SetPinInterruptConfig(PORTE, 5, kPORT_InterruptOrDMADisabled);
#endif

	/*Enable Interrupt Requests*/
	NVIC_SetPriority(PORTA_IRQn, 3);
	EnableIRQ(PORTA_IRQn);
	NVIC_SetPriority(PORTC_IRQn, 3);
	EnableIRQ(PORTC_IRQn);
	NVIC_SetPriority(PORTD_IRQn, 3);
	EnableIRQ(PORTD_IRQn);
}
/*END gpioSetup******************************************************************/


/*FUNCTION**********************************************************************
 *
 * Function Name : adcSetup
 * Description   : This function initializes the ADC. Not complete, but functional
 *
 *END**************************************************************************/

void adcSetup() {

	uint8_t adcAutoCalCount = 0;

	/*ADC settings  ADC configuration either for ADC0 or ADC1*/
	adcUserConfig.clockSource = kADC16_ClockSourceAlt0; //Bus clock
	adcUserConfig.clockDivider = kADC16_ClockDivider4;
	adcUserConfig.enableContinuousConversion = true;
	adcUserConfig.enableAsynchronousClock = false;
	adcUserConfig.enableLowPower = false;
	adcUserConfig.enableHighSpeed = true;
	adcUserConfig.longSampleMode = kADC16_LongSampleCycle24;
	adcUserConfig.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;  //Use external REF
	adcUserConfig.resolution = kADC16_ResolutionSE16Bit;

	/*Set up ADC0*/
	ADC16_Init(ADC0, &adcUserConfig);
	//ADC16_Init(ADC1, &adcUserConfig);

	/*Disable Hardware Triggers on ADCs*/
	ADC16_EnableHardwareTrigger(ADC0, false);  //also disable PDB (programmable delay block)
	//ADC16_EnableHardwareTrigger(ACD1, false);

	/*Disable Hardware Compare Feature on ADC0*/
	ADC16_SetHardwareCompareConfig(ADC0, NULL); //compare function is disabled. COCO flag is set when a conversion completed
	//ADC16_SetHardwareCompareConfig(ADC1, NULL);

	ADC16_SetHardwareAverage(ADC0, kADC16_HardwareAverageCount32);
	//ADC16_SetHardwareAverage(ADC1, kADC16_HardwareAverageCount32);

	while (ADC16_DoAutoCalibration(ADC0) != kStatus_Success){
		adcAutoCalCount++;
		if (adcAutoCalCount == MAX_ADC_CAL){
			break;
		}
	}

	//if (ADC16_DoAutoCalibration(ADC1) != kStatus_Success){
	//PIEZO-BUZZ
	//}

	/*Settings for Oxygen ADC on ADC0 Channel 14 - ADC channel config, set specific channel under a ADC module*/
	adcOxygenConfig.channelNumber = OXY_CHANNEL;
	adcOxygenConfig.enableDifferentialConversion = false;         //NO differential signal
	adcOxygenConfig.enableInterruptOnConversionCompleted = false; //No interrupt when AD conversion completed

	/*Settings for Pressure ADC on ADC0 Channel 9*/
	adcPresConfig.channelNumber = PRES_CHANNEL;
	adcPresConfig.enableDifferentialConversion = false;
	adcPresConfig.enableInterruptOnConversionCompleted = false;

	/*Settings for ADC Power Source on VSS*/
	adcPowerConfig.channelNumber = PWR_CHANNEL;
	adcPowerConfig.enableDifferentialConversion = false;
	adcPowerConfig.enableInterruptOnConversionCompleted = false;

	/*Settings for ADC Ground Source*/
	adcGndConfig.channelNumber = GND_CHANNEL;
	adcGndConfig.enableDifferentialConversion = false;
	adcGndConfig.enableInterruptOnConversionCompleted = false;

	/*Settings for Extra analog input on ADC1 Channel 0 */
	adcExtraConfig.channelNumber = EXTRA_ADC_CH;
	adcExtraConfig.enableDifferentialConversion = false;
	adcExtraConfig.enableInterruptOnConversionCompleted = false;

	//adcExtraDiffConfig.channelNumber = DIFF_ADC_CH;
	//adcExtraDiffConfig.enableDifferentialConversion = true;
	//adcExtraDiffConfig.enableInterruptOnConversionCompleted = false;
}
/*END adcSetup******************************************************************/


/********************************************************************************
 * UART SETUP FUNCTIONS
 ********************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : uartSetup
 * Description   : Set up UART 0 and UART 4
 *
 ******************************************************************************/
void uartSetup(){

	/*CO2 Serial comm settings*/
	UART_GetDefaultConfig(&uartCarbonConfig);
	uartCarbonConfig.parityMode = kUART_ParityEven; //1 parity bit for each byte
	uartCarbonConfig.baudRate_Bps = 38400;
	uartCarbonConfig.enableTx = true;  //C2 TE set
	uartCarbonConfig.enableRx = true;  //C2 RE set

	/*Set up CO2 on UART 0*/
	UART_Init (UART0, &uartCarbonConfig, CLOCK_GetFreq(CO_UART_CLK_SRC)); //UART0 and 1 TX/RX FIFO size: 8

	/*USB Serial comm settings*/
	uartUsbConfig.parityMode = kUART_ParityEven;
	uartUsbConfig.baudRate_Bps = USB_BAUD;
	uartUsbConfig.enableTx = true;
	uartUsbConfig.enableRx = true;

	/*Set up USB on UART 4*/
	UART_Init (UART4, &uartUsbConfig, CLOCK_GetFreq(USB_UART_CLK_SRC)); //UART4 TX/RX FIFO size: 1

	/*Set up BT comm on UART 3*/
	UART_GetDefaultConfig(&uartBtConfig);
	uartBtConfig.parityMode = kUART_ParityDisabled;
	uartBtConfig.baudRate_Bps = 115200;
	uartBtConfig.enableTx = true;
	uartBtConfig.enableRx = true;

	UART_Init (UART1, &uartBtConfig, CLOCK_GetFreq(BT_UART_CLK_SRC)); //UART3 TX/RX FIFO size: 1
}
/*END uartSetup******************************************************************/


/*FUNCTION**********************************************************************
 *
 * Function Name : uartInterruptSetup
 * Description   : Enable Interrupts on UART0 and UART 4
 *
 ******************************************************************************/
void uartInterruptSetup(){

	/*Command CO2 sensor to send data package at 60 Hz*/
	uint8_t sendString[] = "<D321>";//DATA_RATE=3*16+2 = 50 Hz
	uint8_t returnString[10];
	size_t commandLen;

	/*write data rate to CO2 Module*/
	//set the data rate right before UART interrupt is enabled
	commandLen = sizeof(sendString) - 1;

	do {
		UART_WriteBlocking(CO2_UART, sendString, commandLen);

		UART_ReadBlocking(CO2_UART, returnString, commandLen);
	} while(strncmp((const char*)sendString, (const char *)returnString, commandLen) != 0);

	/*Enable Interrupts on UART 0*/
	UART_EnableInterrupts(UART0, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
	NVIC_SetPriority(UART0_RX_TX_IRQn, 0);//
	EnableIRQ(UART0_RX_TX_IRQn);

	/*Enable Interrupts on UART 4*/
//	UART_EnableInterrupts(UART4, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
//	NVIC_SetPriority(UART4_RX_TX_IRQn, 15);
//	EnableIRQ(UART4_RX_TX_IRQn);

	UART_EnableInterrupts(UART1, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
	NVIC_SetPriority(UART1_RX_TX_IRQn, 1);
	EnableIRQ(UART1_RX_TX_IRQn);
}
/*END uartInterruptSetup********************************************************/


/********************************************************************************
 * END OF UART SETUP FUNCTIONS
 ********************************************************************************/


/*FUNCTION**********************************************************************
 *
 * Function Name : i2cSetup
 * Description   : sets up I2C communication with the RH/temp module and the
 * 					digital potentiometer
 *
 ******************************************************************************/
void i2cSetup (){

	/*Settings for RH/Temp i2c*/
	rhCommConfig.enableMaster = true;
	rhCommConfig.glitchFilterWidth = 0U;
	rhCommConfig.baudRate_Bps = RH_BAUD; ///300kHz -> NOTE: When migrating to SHT35, update to 1Mhz

	/*Set up RH/Temp i2c*/
	I2C_MasterInit(RH_I2C, &rhCommConfig, CLOCK_GetFreq(RH_I2C_CLK_SRC));

	/*Settings for lcd i2c*/
	lcdCommConfig.enableMaster = true;
	lcdCommConfig.glitchFilterWidth = 0U;
	lcdCommConfig.baudRate_Bps = LCD_BAUD; //max 400kHz

	/*Set up Digital Pot i2c*/
	I2C_MasterInit(LCD_I2C, &lcdCommConfig, CLOCK_GetFreq(LCD_I2C_CLK_SRC));

}
/*END i2cSetup******************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : spiSetup
 * Description   : Set up SPI communication with DAC (and BlueTooth once implemented)
 *
 ******************************************************************************/
void spiSetup(){

	/*container for DSPI settings, DAC DSPI is in Master Mode*/
	dspi_master_config_t spiDacMasterConfig;//add spiBTMasterConfig

	/*Clock and Transfer Attribute (CTAR) configuration settings*/
	spiDacMasterConfig.whichCtar = kDSPI_Ctar0; //Use CTAR0
	spiDacMasterConfig.ctarConfig.baudRate = DAC_TRANSFER_BAUD;
	spiDacMasterConfig.ctarConfig.bitsPerFrame = DAC_TRANSFER_BITS; //8 bits per frame
	spiDacMasterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveLow;
	spiDacMasterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge; //First Data bit available on first clock edge
	spiDacMasterConfig.ctarConfig.direction = kDSPI_MsbFirst; //Most significant bit first
	spiDacMasterConfig.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U/DAC_TRANSFER_BAUD;
	spiDacMasterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U/DAC_TRANSFER_BAUD;
	spiDacMasterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U/DAC_TRANSFER_BAUD;

	/*DSPI Settings*/
	spiDacMasterConfig.whichPcs = DAC_SPI_CS_INIT;//Use DAC Chip select
	spiDacMasterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;
	spiDacMasterConfig.enableContinuousSCK = true; //use continuous selection format pcs remains asserted between transfers
	spiDacMasterConfig.enableModifiedTimingFormat = false;
	spiDacMasterConfig.enableRxFifoOverWrite = false;
	spiDacMasterConfig.samplePoint = kDSPI_SckToSin0Clock;

	/*Direct Memory Access SPI Setup DAC SPI*/
	DSPI_MasterInit(DAC_SPI, &spiDacMasterConfig, CLOCK_GetFreq(DSPI2_CLK_SRC));

	/* Enable AD5624R On-Chip Voltage Reference VREF
	 * NOTE: Code assumes 8-bit transfer size */
	dspi_transfer_t vRefSetup;
	uint8_t xferData[DAC_TRANSFER_CYCLES]; //tx buffer size 3, fifo
	uint8_t recvData[DAC_TRANSFER_CYCLES] = {0U}; //rx buffer size 3, fifo

	xferData[0] = 0x38;  //the register value to turn on internal reference
	xferData[1] = 0x00;
	xferData[2] = 0x01;


	vRefSetup.txData = xferData;
	vRefSetup.rxData = recvData;
	vRefSetup.dataSize = DAC_TRANSFER_CYCLES;
	vRefSetup.configFlags = kDSPI_MasterCtar0 | DAC_SPI_CS_XFER | kDSPI_MasterPcsContinuous;

	DSPI_MasterTransferBlocking(DAC_SPI, &vRefSetup);

	/*SPI BT Set up*/
	//TODO

}
/*END spiSetup******************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : timerSetup
 * Description   : Sets up Periodic Interrupt Timer. PIT1 is set to 1 us. Timer
 * 					used to control delay, though can be used as a general purpose timer
 *
 ******************************************************************************/
void timerSetup()
{
	//periodic interrupt timer

	pit_config_t pitConfig;
	pitConfig.enableRunInDebug = true;
	PIT_Init(PIT, &pitConfig);

	/* 定时器0用于系统调度 */
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, MSEC_TO_COUNT(1, CLOCK_GetFreq(kCLOCK_BusClk)));
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

	/*Enable Interrupt Request on PIT0*/
	NVIC_SetPriority(PIT0_IRQn, 2);
	EnableIRQ(PIT0_IRQn);
	PIT_StartTimer(PIT, kPIT_Chnl_0);

	/*set up PIT1 for piezoBuzz*/
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_1, USEC_TO_COUNT(1, CLOCK_GetFreq(kCLOCK_BusClk)));//an interrupt every 1us
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_1, kPIT_TimerFlag);
	PIT_EnableInterrupts(PIT, kPIT_Chnl_1, kPIT_TimerInterruptEnable);

	/*Enable Interrupt Request on PIT1*/
	NVIC_SetPriority(PIT1_IRQn, 13);
	EnableIRQ(PIT1_IRQn);

	/*set up PIT2, used for time SD card time stamp, interval ms --TO check LPO, could be more accurate*/
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_2, USEC_TO_COUNT(1000, CLOCK_GetFreq(kCLOCK_BusClk)));
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_2, kPIT_TimerFlag);
	PIT_EnableInterrupts(PIT, kPIT_Chnl_2, kPIT_TimerInterruptEnable);

	/*Enable Interrupt Request on PIT2*/
	NVIC_SetPriority(PIT2_IRQn, 14);
	EnableIRQ(PIT2_IRQn);

//	//For RTC debug
//	PIT_SetTimerPeriod(PIT, kPIT_Chnl_3, USEC_TO_COUNT(1000, CLOCK_GetFreq(kCLOCK_BusClk)));
//	PIT_ClearStatusFlags(PIT, kPIT_Chnl_3, kPIT_TimerFlag);
//	PIT_EnableInterrupts(PIT, kPIT_Chnl_3, kPIT_TimerInterruptEnable);
//
//	/*Enable Interrupt Request on PIT2*/
//	NVIC_SetPriority(PIT3_IRQn, 12);
//	EnableIRQ(PIT3_IRQn);
}
/*END timerSetup******************************************************************/



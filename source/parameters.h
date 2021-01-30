/*******************************************************************************
 * parameters.h
 *
 *  Created on: June 21, 2017
 *      Author: Daniel Borisov
 *
 *      This header file contains definitions for:
 *      - Gas Concentration and Flow Information
 *      - Internal Reference Parameters
 *      - Comm Protocol Parameter
 *      - I2C addresses
 *      - Peripheral Name Masks
 *      -
 *  REVISION HISTORY: (to be filled once code reaches first 'complete' state, or after Dan leaves)
 *      Revisions made: "# define SHT35" uncommented
 *      Date: 2018-07-31
 *      Author: BK
 *
 *      Revisions made: Define RH_MEASURE and TEMP_MEASURE under SHT35 if def to facilitate overloading
 *      	of functions in humidity.cpp
 *      Date: 2018-07-13
 *      Author: BK
 ******************************************************************************/

#ifndef SOURCE_PARAMETERS_H_
#define SOURCE_PARAMETERS_H_

#include <stdint.h>

/*-----------------------------------------------------------*/
#define SHT35 //<--- Uncommented now that the new boards are in
/*-----------------------------------------------------------*/

/* Define Variable Parameters for Gas and Concentrations and Flow*/
#define CAL_GAS "<C0500>" //Calibration gas CO2 concentration; 05.00 %
#define DATA_RATE "<D3C1>" //this parameter wasn't used, the real rate is 50Hz, i.e. "<D321>"
#define FLOW_MAX 23.08885384f // Litres/sec, 16.52833985-(-15.40805973)
#define FLOW_OFFSET 11.0633988f //15.40805973 //according to the lookup table
#define CARB_MAX 0.1f // %, for DAC Outputs
#define OXY_MAX 0.25f // %, for DAC Outputs
#define EXTRA_MAX 100.0f // -, for DAC Outputs (set to maximum value of whatever you're trying to read)
#define OXY_SLOPE 76//81.52844328
#define OXY_INTERCEPT 7.6//8.152844328 //For conversion of voltage to mmHg
#define OXY_CAL_LO 0.20949 //oxygen span lower boundary
#define OXY_CAL_HI 0.209505 //oxygen span higher boundary----standard value is 0.2095

/* Define Internal Reference Parameters */
#define PRES_DAC_CH 0
#define CARB_DAC_CH 1
#define OXY_DAC_CH 2
#define EXTRA_DAC_CH 3
//---
#define RH_I2C_CH 0
#define LCD_I2C_CH 1 //Digital Potentiometer
//---
#define DAC_SPI_CS_INIT kDSPI_Pcs0
#define DAC_SPI_CS_XFER kDSPI_MasterPcs0
//---
#define DAC_RESOLUTION 12U //In bits
#define DAC_RESOLUTION_COUNT 16384U //2^DAC_RESOLUTION (depends on DAC being used), 4096 for 12-bit
//---
#define ADC_RESOLUTION 16U
#define ADC_RESOLUTION_COUNT 65536U //2^ADC_RESOLUTION
//---
#define UP_DIR 0
#define DOWN_DIR 1
#define O_POT 0
#define PRES_POT 1
#define O_POT_INIT_STEPS     65
#define PRES_POT_INIT_STEPS  8
//---
#define SIZE_OF_SERIAL_RECEIVED 16
//---
#define CONC_COMMAND 1U
#define CARB_COMMAND 2U
#define FLOW_COMMAND 3U
#define BT_COMMAND 4U
#define SD_COMMAND 5U
#define OXY_COMMAND 6U
#define MUSIC_COMMAND 7U


//Pressure signal gets 6mV window BK - 2018-02-16
#define PRES_ADC_UPPER_VOLTAGE 1.5004 //1.251 BK - 2018/02/08 Upper hysteresis level of PRES Zero Voltage (account for quantization error)
#define PRES_ADC_LOWER_VOLTAGE 1.4996 //1.245098039 is the voltage for zero flow
//todo Do we need a pressure offset? How consistent is the offset between different Q-Track Units
//---
#define CARBON_BUFFER_SIZE 64
#define USB_BUFFER_SIZE 64
#define USB_PACKET_BUFFER 7

#ifdef SHT35

#define RH_PIT_USEC_COUNT 200000U //used to be 20000us (20ms) between samples, now 200ms

#define RH_BAUD  360000//950000, 950kHz for SHT35
#define RH_TRANSFER_LENGTH 0U//2 for SHT35
#define RH_RECEIVE_LENGTH 4U// 6 for SHT35, a CRC for each 16-bit data

#define RH_REQ_CMD 0x24

#define RH_ADDRESS 0x27//  0b01000100 for SHT35

#define RH_STOP_FLAG_MASK kI2C_TransferDefaultFlag

#define T_REQ_CMD 0
#define RH_MEASURE (uint8_t)0U
#define TEMP_MEASURE (uint8_t)1U

#else //SHT25

#define RH_PIT_USEC_COUNT 100000U //100ms between samples

#define RH_BAUD 350000 //350kHz
#define RH_TRANSFER_LENGTH 1
#define RH_RECEIVE_LENGTH 3


#define RH_REQ_CMD 0b11100101
#define T_REQ_CMD 0b11100011

#define RH_ADDRESS 0b0100000

#define RH_STOP_FLAG_MASK kI2C_TransferNoStopFlag

#define RH_MEASURE (uint8_t)0U
#define TEMP_MEASURE (uint8_t)1U

#endif

/* Define Comm Protocol Parameters  */
#define DAC_TRANSFER_BAUD 1000000U//24000000U
#define DAC_SPI_CS kDSPI_MasterPcs0
#define DAC_TRANSFER_BITS 8U
#define DAC_TRANSFER_LENG 24U
#define DAC_TRANSFER_CYCLES 3U  // -> DAC_TRANSFER_LENG/DAC_TRANSFER_BITS
#define DAC_WRITE_CMD_BYTE 0x18 // 0bxx011xxx
#define DAC_FLOW_ADDR 0x00
#define DAC_CARB_ADDR 0x01
#define DAC_OXY_ADDR 0x02
#define DAC_EXTRA_ADDR 0x03
//---digital pot AD5170 ---> change to LCD
#define LCD_BAUD 360000
#define LCD_TRANSFER_LENGTH 2  //need to change for LCD
//#define POT_INSTR_CMD 0b00001000
//#define POT_FUSE_A_CMD 0b0010000
//#define POT_FUSE_B_CMD 0b1010000
//---
#define USB_BAUD 1500000//1500000
//channels for ADC
#define OXY_CHANNEL 14
#define PRES_CHANNEL 9
#define PWR_CHANNEL 15
#define GND_CHANNEL 8
#define EXTRA_ADC_CH 16
#define DIFF_ADC_CH 3

/* Define I2C addresses */
#define LCD_ADDRESS 0x3C//0b00111100 with SA0 connected to GND

/* Define Peripheral Name Masks */
#define CO2_UART UART0
#define USB_UART UART4 //Added 2018-07-04 BK
#define DAC_SPI SPI2
#define LCD_I2C I2C0
#define LCD_I2C_CLK_SRC I2C0_CLK_SRC
#define RH_I2C I2C1                                 //Use I2C1 for RH_I2C
#define RH_I2C_CLK_SRC I2C1_CLK_SRC
#define CO_UART_CLK_SRC UART0_CLK_SRC
#define USB_UART_CLK_SRC UART4_CLK_SRC
#define BT_UART_CLK_SRC UART1_CLK_SRC
#define LED_IO GPIOB
#define PRES_CS_IO GPIOC
#define INC_POT_IO GPIOC
#define UD_POT_IO GPIOD
#define O2_CS_IO GPIOD
#define PIEZO_IO GPIOD
#define CAL_IO GPIOD
#define SD_DETECT_IO GPIOD////YC_forSD

/* Define Pin Mappings */
#define A_EZP_CLK A(0)
#define A_EZP_DI A(1)
#define A_EZP_DO A(2)
#define A_EZP_CS_b A(4)
#define A_BT_RESET A(3)//needs to be configured as JTAG_TMS
#define A_BT_PAIR_BUTTON A(12)
#define A_START_STOP_BUTTON A(13)
#define A_PRES_ZERO_BUTTON A(14)
#define A_PTA15 A(15)
#define A_PTA18 A(18)
#define A_PTA19 A(19)

#define B_ADC0_SE18_AGND B(0)
#define B_ADC0_SE19_PRES_SIG B(1)
#define B_I2C0_SCL_LCD B(2)
#define B_I2C0_SDA_LCD B(3)
#define B_UART1_RX C(3)
#define B_UART1_TX C(4)
#define B_UART0_RX_CO2TX B(16)
#define B_UART0_TX_CO2RX B(17)
//#define B_GREEN_LED B(18)
//#define B_RED_LED B(19)
#define B_SPI2_PCS0_DAC B(20)
#define B_SPI2_SCK_DAC B(21)
#define B_SPI2_SOUT_DAC B(22)

#define C_ADC0_SE14_O2_SIG C(0)
#define C_ADC0_SE15_PWR C(1)
#define C_SPI0_PCS0 C(4)
#define C_SPI0_SCK C(5)
#define C_SPI0_SOUT C(6)
#define C_SPI0_SIN C(7)
#define C_I2C1_SCL_RH_CLK C(10)
#define C_I2C1_SDA_RH_DAT C(11)
#define C_UART4_RTS_USB_CTS C(12)
#define C_UART4_CTS_USB_RTS C(13)
#define C_UART4_RX_USB_TX C(14)
#define C_UART4_TX_USB_RX C(15)
#define C_ZERO_CO2_BUTTON C(16)
#define C_PRES_ZERO_CS C(17)
#define C_INC_CTRL C(18)

#define D_O2_SPAN_CS D(2)
#define D_UD_POT_CTRL D(3)
#define D_O2_CAL_BUTTON D(4)
#define D_PIEZO_CTRL D(5)
#define D_CAL_HEADER D(6)
#define D_SD_DETECT D(7)//YC_forSD

//SD pin definition
#define E_SDHC0_D1 E(0)
#define E_SDHC0_D0 E(1)
#define E_SDHC0_DCLK E(2)
#define E_SDHC0_CMD E(3)
#define E_SDHC0_D3 E(4)
#define E_SDHC0_D2 E(5)

///////////////////

#define OUTPUT_PORTA_PINS (uint32_t)(A_EZP_DO | A_BT_RESET | A_BT_PAIR_BUTTON | A_START_STOP_BUTTON | A_PRES_ZERO_BUTTON)
#define INPUT_PORTA_PINS (uint32_t)(A_EZP_CLK | A_EZP_DI | A_EZP_CS_b | A_PTA15 | A_PTA18 | A_PTA19)
#define UNDEFINED_PORTA_PINS (uint32_t)0
#define UNUSED_PORTA_PINS (uint32_t)((~(OUTPUT_PORTA_PINS | INPUT_PORTA_PINS | UNDEFINED_PORTA_PINS)) & PORTA_VALID_PINS)

///////////////////

#define OUTPUT_PORTB_PINS (uint32_t)(B_UART3_TX | B_UART0_TX_CO2RX | B_SPI2_SOUT_DAC | B_SPI2_SCK_DAC | B_SPI2_PCS0_DAC | B_GREEN_LED | B_RED_LED)
#define INPUT_PORTB_PINS (uint32_t)(B_ADC0_SE18_AGND | B_ADC0_SE19_PRES_SIG | B_UART3_RX | B_UART0_RX_CO2TX)
#define UNDEFINED_PORTB_PINS (uint32_t)(B_I2C0_SCL_DIG_POT | B_I2C0_SDA_DIG_POT)
#define UNUSED_PORTB_PINS (uint32_t)((~(OUTPUT_PORTB_PINS | INPUT_PORTB_PINS | UNDEFINED_PORTB_PINS)) & PORTB_VALID_PINS)

///////////////////

#define OUTPUT_PORTC_PINS (uint32_t)(C_SPI0_SCK | C_SPI0_PCS0 | C_SPI0_SOUT | C_UART4_RTS_USB_CTS | C_UART4_TX_USB_RX | C_PRES_ZERO_CS | C_INC_CTRL)
#define INPUT_PORTC_PINS (uint32_t)(C_ADC0_SE14_O2_SIG | C_ADC0_SE15_PWR | C_SPI0_SIN | C_UART4_RX_USB_TX | C_UART4_CTS_USB_RTS | C_ZERO_CO2_BUTTON)
#define UNDEFINED_PORTC_PINS (uint32_t)(C_I2C1_SCL_RH_CLK | C_I2C1_SDA_RH_DAT)
#define UNUSED_PORTC_PINS (uint32_t)((~(OUTPUT_PORTC_PINS | INPUT_PORTC_PINS | UNDEFINED_PORTC_PINS)) & PORTC_VALID_PINS)

///////////////////

#define OUTPUT_PORTD_PINS (uint32_t)(D_O2_SPAN_CS | D_UD_POT_CTRL | D_PIEZO_CTRL)
#define INPUT_PORTD_PINS (uint32_t)(D_O2_CAL_BUTTON | D_CAL_HEADER | D_SD_DETECT)
#define UNDEFINED_PORTD_PINS (uint32_t)0
#define UNUSED_PORTD_PINS (uint32_t)((~(OUTPUT_PORTD_PINS | INPUT_PORTD_PINS | UNDEFINED_PORTD_PINS)) & PORTD_VALID_PINS)

///////////////////

#define OUTPUT_PORTE_PINS (uint32_t)(E_SDHC0_DCLK)
#define INPUT_PORTE_PINS (uint32_t)0
#define UNDEFINED_PORTE_PINS (uint32_t)(E_SDHC0_D1 | E_SDHC0_D0 | E_SDHC0_CMD | E_SDHC0_D3 | E_SDHC0_D2)
#define UNUSED_PORTE_PINS (uint32_t)((~(OUTPUT_PORTE_PINS | INPUT_PORTE_PINS | UNDEFINED_PORTE_PINS)) & PORTE_VALID_PINS)

///////////////////

/* Define Pin MUX */

#define A_EZP_CLK_MUX kPORT_MuxAlt7
#define A_EZP_DI_MUX kPORT_MuxAlt7
#define A_EZP_DO_MUX kPORT_MuxAlt7
#define A_EZP_CS_b_MUX kPORT_MuxAlt7
#define A_BT_RESET_MUX kPORT_MuxAsGpio
#define A_BT_PAIR_BUTTON_MUX kPORT_MuxAsGpio
#define A_START_STOP_BUTTON_MUX kPORT_MuxAsGpio
#define A_PRES_ZERO_BUTTON_MUX kPORT_MuxAsGpio
#define A_PTA15_MUX kPORT_MuxAsGpio
#define A_PTA18_MUX kPORT_MuxAsGpio
#define A_PTA19_MUX kPORT_MuxAsGpio

#define B_ADC0_SE18_AGND_MUX kPORT_PinDisabledOrAnalog
#define B_ADC0_SE19_PRES_SIG_MUX kPORT_PinDisabledOrAnalog
#define B_I2C0_SCL_DIG_POT_MUX kPORT_MuxAlt2
#define B_I2C0_SDA_DIG_POT_MUX kPORT_MuxAlt2
#define B_UART1_RX_MUX kPORT_MuxAlt3
#define B_UART1_TX_MUX kPORT_MuxAlt3
#define B_UART0_RX_CO2TX_MUX kPORT_MuxAlt3
#define B_UART0_TX_CO2RX_MUX kPORT_MuxAlt3
#define B_GREEN_LED_MUX kPORT_MuxAsGpio
#define B_RED_LED_MUX kPORT_MuxAsGpio
#define B_SPI2_PCS0_DAC_MUX kPORT_MuxAlt2
#define B_SPI2_SCK_DAC_MUX kPORT_MuxAlt2
#define B_SPI2_SOUT_DAC_MUX kPORT_MuxAlt2

#define C_ADC0_SE14_O2_SIG_MUX kPORT_PinDisabledOrAnalog
//#define C_ADC0_SE15_PWR_MUX kPORT_PinDisabledOrAnalog
#define C_SPI0_PCS0_MUX kPORT_MuxAlt2
#define C_SPI0_SCK_MUX kPORT_MuxAlt2
#define C_SPI0_SOUT_MUX kPORT_MuxAlt2
#define C_SPI0_SIN_MUX kPORT_MuxAlt2
#define C_I2C1_SCL_RH_CLK_MUX kPORT_MuxAlt2
#define C_I2C1_SDA_RH_DAT_MUX kPORT_MuxAlt2
#define C_UART4_RTS_USB_CTS_MUX kPORT_MuxAlt3
#define C_UART4_CTS_USB_RTS_MUX kPORT_MuxAlt3
#define C_UART4_RX_USB_TX_MUX kPORT_MuxAlt3
#define C_UART4_TX_USB_RX_MUX kPORT_MuxAlt3
#define C_ZERO_CO2_BUTTON_MUX kPORT_MuxAsGpio
#define C_PRES_ZERO_CS_MUX kPORT_MuxAsGpio
#define C_INC_CTRL_MUX kPORT_MuxAsGpio

#define D_O2_SPAN_CS_MUX kPORT_MuxAsGpio
#define D_UD_POT_CTRL_MUX kPORT_MuxAsGpio
#define D_O2_CAL_BUTTON_MUX kPORT_MuxAsGpio
#define D_PIEZO_CTRL_MUX kPORT_MuxAsGpio
#define D_CAL_HEADER_MUX kPORT_MuxAsGpio
#define D_SD_DETECT_MUX kPORT_MuxAsGpio//YC_forSD connected to the DETECT_SWITCH pin of the SD Card holder
                                       //Set as GPIO
//YC_forSD SDHC pin mux
#define E_SDHC0_D1_MUX kPORT_MuxAlt4
#define E_SDHC0_D0_MUX kPORT_MuxAlt4
#define E_SDHC0_DCLK_MUX kPORT_MuxAlt4
#define E_SDHC0_CMD_MUX kPORT_MuxAlt4
#define E_SDHC0_D3_MUX kPORT_MuxAlt4
#define E_SDHC0_D2_MUX kPORT_MuxAlt4

  /*!< Macro to enable all interrupts. */
#define EnableInterrupts asm(" CPSIE i");

  /*!< Macro to disable all interrupts. */
#define DisableInterrupts asm(" CPSID i");

/* Define User-Friendly Port Definitions */
#define PORTA_VALID_PINS (uint32_t)0x000CF01F	// 0-4, 12-15, 18-19 ; 00000000000011001111000000011111
#define PORTB_VALID_PINS (uint32_t)0x00FF0E0F   // 0-3, 9-11, 16-23  ; 00000000111111110000111000001111
#define PORTC_VALID_PINS (uint32_t)0x0007FFFF   // 0-18				 ; 00000000000001111111111111111111
#define PORTD_VALID_PINS (uint32_t)0x000000FF   // 0-7				 ; 00000000000000000000000011111111
#define PORTE_VALID_PINS (uint32_t)0x0000003F   // 0-5				 ; 00000000000000000000000000111111

#define A(X) (uint32_t)((1U<<X) & PORTA_VALID_PINS)
#define B(X) (uint32_t)((1U<<X) & PORTB_VALID_PINS)
#define C(X) (uint32_t)((1U<<X) & PORTC_VALID_PINS)
#define D(X) (uint32_t)((1U<<X) & PORTD_VALID_PINS)
#define E(X) (uint32_t)((1U<<X) & PORTE_VALID_PINS)

/* Define Note Frequencies */
#define A_FLAT 415
#define A_NOTE 440//440, 500 if use uSec
#define B_FLAT 466
#define B_NOTE 494
#define C_NOTE 523//original value: 523, 250 if use mSec
#define C_SHARP 554
#define D_NOTE 587
#define D_SHARP 622
#define E_NOTE 659
#define F_NOTE 698
#define F_SHARP 740
#define G_NOTE 784

#define MS_CYCLES 125000 //numbers of instructions of the MCU in 1 ms
#define MAX_POT_OPP_MOVE 5U
#define POT_INITIAL      2U

//YC_forSD
/*Data block count accessed in card*/
#define DATA_BLOCK_COUNT (5U)
/*Start data block number accessed in card */
#define DATA_BLOCK_START (2U)
/*Data buffer size*/
#define DATA_BUFFER_SIZE (FSL_SDMMC_DEFAULT_BLOCK_SIZE * DATA_BLOCK_COUNT)

//LCD routine
#define LCD_INIT           0U
#define LCD_SD_DETECTING   1U
#define LCD_SD_DETECTED    2U
#define LCD_SD_NOTDETECTED 3U
#define LCD_INSERT_SD      4U
#define LCD_SD_COLLECT     5U
#define LCD_SD_REMOVED     6U
#define LCD_PUMPSET        7U
#define LCD_STARTUP_CAL    8U
//#define LCD_INIT_DONE      8U
#define LCD_CO2_ZERO_CAL   9U
#define LCD_CO2_SPAN_CAL   10U
#define LCD_FLOW_ZERO_CAL  11U
#define LCD_O2_SPAN_CAL    12U
#define LCD_BT_PAIR        13U

//LCD Data
#define LCD_CO2  0U
#define LCD_O2   1U
#define LCD_FLOW 2U
#define LCD_RH   3U
#define LCD_TEMP 4U
#define LCD_BARO 5U

#define EPSILON 0.000000001

#define DAT_COUNT_MAX_LCD 15 //LCD updates data about every 600ms
#define LCD_ON_MAX 100 //on 250 for 50 seconds

#define MAX_ADC_CAL 5U

//for O2 autoCal, unit is minutes
#define FIRSTSTAGE_CALCOUNT 7    //first stage is defined as within 42 minutes of operation, cal every 6 min
#define SECONDSTAGE_CALCOUNT 4   //second stage is defined as between 42 and 102 minutes of operation, cal every 15 min
#define INTVL1 6
#define INTVL2 15
#define INTVL3 30

#endif /* SOURCE_PARAMETERS_H_ */
